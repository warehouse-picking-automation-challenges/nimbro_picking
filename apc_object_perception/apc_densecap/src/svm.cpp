// Encapsulates liblinear SVM training
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "svm.h"

#include <chrono>

#include <linear.h>

namespace apc_densecap
{

static void dummy(const char*)
{
}

struct SVMStorage
{
	std::vector<feature_node*> rowPtrs;
	std::vector<std::vector<feature_node>> rows;
	std::vector<int> y;

	unsigned int nRows = 0;
	int maxIndex = 0;

	void addSample(const Eigen::VectorXf& sample, int label, double biasScale)
	{
		if(nRows >= rows.size())
			rows.emplace_back();

		auto& storage = rows[nRows];
		nRows++;

		storage.clear();
		storage.reserve(sample.size()+2);

		for(unsigned int i = 0; i < sample.size(); ++i)
		{
			if(sample[i] == 0.0)
				continue;

			feature_node node;
			node.index = i+1;
			node.value = sample[i];
			storage.push_back(node);
		}

		feature_node bias;
		bias.index = sample.size()+1;
		bias.value = biasScale;
		storage.push_back(bias);

		feature_node sentinel;
		sentinel.index = -1;
		storage.push_back(sentinel);

		maxIndex = std::max<int>(maxIndex, sample.size()+1);

		y.push_back(label);
	}

	int numFeatures()
	{
		return maxIndex + 1;
	}

	void clear()
	{
		rowPtrs.clear();
		nRows = 0;
		maxIndex = 0;
		y.clear();
	}
};

// Use thread_local storage to prevent frequent reallocations
static thread_local SVMStorage g_storage;

bool SVMModel::train(
	const SampleDB& samples, const std::vector<std::string>& presentItems,
	const std::string& target, const Parameters& params,
	const boost::filesystem::path& exclude)
{
	problem prob;

	auto start = std::chrono::high_resolution_clock::now();

	g_storage.clear();

	// Insert positive examples
	for(auto& file : samples.files())
	{
		// CV: exclude training data from this file
		if(file.first == exclude)
			continue;

		auto it = file.second->samples.find(target);
		if(it == file.second->samples.end())
			continue;

		for(auto& vec : it->second)
			g_storage.addSample(vec.array(), 1, params.biasScale);
	}

	if(g_storage.nRows == 0)
	{
		// no positive examples :-(
		return false;
	}

	// Insert negative examples
	for(auto& file : samples.files())
	{
		// CV: exclude training data from this file
		if(file.first == exclude)
			continue;

		for(auto& pair : file.second->samples)
		{
			// skip positive examples
			if(pair.first == target)
				continue;

			// if we want to tailor, skip examples of objects which cannot occur
			if(params.tailor && pair.first != "negative")
			{
				auto it = std::find(presentItems.begin(), presentItems.end(), pair.first);
				if(it == presentItems.end())
					continue;
			}

			for(auto& vec : pair.second)
				g_storage.addSample(vec.array(), -1, params.biasScale);
		}
	}

	g_storage.rowPtrs.resize(g_storage.nRows);
	for(unsigned int i = 0; i < g_storage.nRows; ++i)
		g_storage.rowPtrs[i] = g_storage.rows[i].data();

	prob.n = g_storage.numFeatures();
	prob.l = g_storage.rowPtrs.size();
	prob.x = g_storage.rowPtrs.data();
	prob.y = g_storage.y.data();

	printf("SVM training: %d samples, %d features per sample\n", prob.l, prob.n);

	parameter param;
// 	param.solver_type = L2R_L2LOSS_SVC_DUAL;
// 	param.solver_type = L2R_L2LOSS_SVC;
	param.solver_type = L2R_L1LOSS_SVC_DUAL;
	param.C = params.C;
	param.eps = 0.1;
	param.nr_weight = 0;
	param.weight_label = 0;
	param.weight = 0;

// 	printf("Training with %u samples\n", g_storage.nRows);
	// mute liblinear
	set_print_string_function(&dummy);

	model* m = ::train(&prob, &param);
	if(!m)
	{
		fprintf(stderr, "liblinear training failed!\n");
		return false;
	}

	m_weights = Eigen::VectorXd::Map(m->w, m->nr_feature).cast<float>();
	m_biasScale = params.biasScale;
	free_and_destroy_model(&m);

	auto end = std::chrono::high_resolution_clock::now();
	printf("took %ld\n", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

	return true;
}

float SVMModel::score(const Eigen::VectorXf& featureVector)
{
	return m_weights.head(featureVector.size()).dot(featureVector) + m_weights.tail(1)[0] * m_biasScale;
}



}
