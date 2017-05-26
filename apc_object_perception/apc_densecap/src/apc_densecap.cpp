// Main entry point for apc_densecap
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_densecap/apc_densecap.h>

#include <ros/console.h>
#include <ros/package.h>

#ifndef APC_DENSECAP_MOCK

extern "C"
{
#  include <lualib.h>
#  include <lauxlib.h>
#  include <luaT.h>
}

#  include <TH/THTensor.h>

#  include <pcl/common/transforms.h>

#  include "sample_db.h"
#  include "svm.h"
#  include "proposal_generator.h"

#endif

namespace apc_densecap
{

class APCDenseCapPrivate
{
public:
#ifndef APC_DENSECAP_MOCK
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> ResultMatrix;

	void callLUA(
		const APCDenseCap::PointCloud::ConstPtr& cloud, const APCDenseCap::NormalCloud::ConstPtr& normals, const Eigen::Affine3f& cam2Shelf,
		ResultMatrix* features, ResultMatrix* boxes
	);

	std::string lua_traceback()
	{
		luaL_traceback(L, L, NULL, 0);
		return lua_tostring(L, -1);
	}

	static void handleTorchError(const char* msg, void* data)
	{
		APCDenseCapPrivate* d = reinterpret_cast<APCDenseCapPrivate*>(data);

		ROS_ERROR("Torch error: '%s'\n", msg);
		ROS_ERROR("LUA traceback:\n%s", d->lua_traceback().c_str());
		std::abort();
	}

	static void handleTorchArgError(int idx, const char* msg, void* data)
	{
		APCDenseCapPrivate* d = reinterpret_cast<APCDenseCapPrivate*>(data);

		ROS_ERROR("Torch argument %d error: '%s'\n", idx, msg);
		ROS_ERROR("LUA traceback:\n%s", d->lua_traceback().c_str());
		std::abort();
	}

	void prepareLUA()
	{
		THSetErrorHandler(&handleTorchError, this);
		THSetArgErrorHandler(&handleTorchArgError, this);
	}

	lua_State* L = 0;

	Parameters params;
	SampleDB sampleDB;

	SVMModel svm;

	std::map<std::string, SVMModel> multiSVM;
#endif

	cv::Mat_<uint8_t> binMask;
	cv::Rect binRect;
};

#ifndef APC_DENSECAP_MOCK
void APCDenseCapPrivate::callLUA(
	const APCDenseCap::PointCloud::ConstPtr& cloud, const APCDenseCap::NormalCloud::ConstPtr& normals, const Eigen::Affine3f& cam2Shelf,
	ResultMatrix* features, ResultMatrix* boxes)
{
	if((int)cloud->width != binMask.cols || (int)cloud->height != binMask.rows)
	{
		throw std::runtime_error("bin mask does not match cloud dimensions - did you forget to call setBinMask()?");
	}

	prepareLUA();

	APCDenseCap::PointCloud::Ptr cloudInShelf(new APCDenseCap::PointCloud);
	pcl::transformPointCloud(*cloud, *cloudInShelf, cam2Shelf);

	auto proposals = generateProposals(cloud, normals, binMask, ProposalParameters());

	THFloatTensor* proposalTensor = THFloatTensor_newWithSize2d(
		proposals.rows(), proposals.cols()
	);
	memcpy(
		THFloatTensor_data(proposalTensor), proposals.data(),
		sizeof(float) * proposals.rows() * proposals.cols()
	);

	// Extract RGB float image
	THFloatTensor* tensor = THFloatTensor_newWithSize3d(
		3, cloud->height, cloud->width
	);
	float* rgbData = THFloatTensor_data(tensor);

	{
		int CHANNEL_STRIDE = cloud->width * cloud->height;

		for(unsigned int y = 0; y < cloud->height; ++y)
		{
			for(unsigned int x = 0; x < cloud->width; ++x)
			{
				auto& p = (*cloud)(x,y);

				rgbData[0 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.r;
				rgbData[1 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.g;
				rgbData[2 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.b;
			}
		}
	}

	// Call LUA code
	{
		lua_getglobal(L, "run_image");

		luaT_pushudata(L, tensor, "torch.FloatTensor");
		luaT_pushudata(L, proposalTensor, "torch.FloatTensor");

		if(lua_pcall(L, 2, 1, 0) != 0)
		{
			ROS_ERROR("LUA error:\n%s", lua_tostring(L, -1));
			throw std::runtime_error("LUA error");
		}

		int outIdx = lua_gettop(L);

		THFloatTensor* thboxes = (THFloatTensor*)luaT_getfieldcheckudata(L, outIdx, "boxes", "torch.FloatTensor");
		THFloatTensor* thfeatures = (THFloatTensor*)luaT_getfieldcheckudata(L, outIdx, "features", "torch.FloatTensor");

		ROS_INFO("Got %ldx%ld box data and %ldx%ld features",
			THFloatTensor_size(thboxes, 0), THFloatTensor_size(thboxes, 1),
			THFloatTensor_size(thfeatures, 0), THFloatTensor_size(thfeatures, 1)
		);

		*boxes = ResultMatrix::Map(
			THFloatTensor_data(thboxes),
			THFloatTensor_size(thboxes, 0), THFloatTensor_size(thboxes, 1)
		);
		*features = ResultMatrix::Map(
			THFloatTensor_data(thfeatures),
			THFloatTensor_size(thfeatures, 0), THFloatTensor_size(thfeatures, 1)
		);

		lua_settop(L, outIdx-1);

		lua_gc(L, LUA_GCCOLLECT, 0);
	}
}
#endif

APCDenseCap::APCDenseCap()
 : m_d(new APCDenseCapPrivate)
{
}

APCDenseCap::~APCDenseCap()
{
#ifndef APC_DENSECAP_MOCK
	THSetErrorHandler(0, 0);
	THSetArgErrorHandler(0, 0);

	if(m_d->L)
		lua_close(m_d->L);
#endif
}

void APCDenseCap::initialize(const std::string& datasetPath, int gpu)
{
#ifndef APC_DENSECAP_MOCK
	// Create LUA interpreter
	m_d->L = luaL_newstate();
	if(!m_d->L)
		throw std::runtime_error("Could not create LUA instance");

	// Load LUA standard libraries
	luaL_openlibs(m_d->L);

	m_d->prepareLUA();

	// Modify LUA path to include densecap
	{
		lua_getglobal(m_d->L, "package");
		lua_getfield(m_d->L, -1, "path");

		std::string path = lua_tostring(m_d->L, -1);
		path += ";" + ros::package::getPath("apc_densecap") + "/densecap/?.lua";

		lua_pop(m_d->L, 1);

		lua_pushstring(m_d->L, path.c_str());
		lua_setfield(m_d->L, -2, "path");

		lua_pop(m_d->L, 1);
	}

	ROS_DEBUG("Running apc.lua...");
	std::string scriptFile = ros::package::getPath("apc_densecap") + "/lua/apc.lua";

	if(luaL_dofile(m_d->L, scriptFile.c_str()) != 0)
	{
		ROS_ERROR("Could not execute '%s'", scriptFile.c_str());
		ROS_ERROR("LUA error:\n%s", lua_tostring(m_d->L, -1));
		throw std::runtime_error("Could not execute LUA script");
	}

	ROS_DEBUG("Loading pretrained model...");
	{
		lua_getglobal(m_d->L, "load_model");

		std::string model = ros::package::getPath("apc_densecap") + "/densecap/data/models/densecap/densecap-pretrained-vgg16.t7";
		lua_pushstring(m_d->L, model.c_str());
		lua_pushinteger(m_d->L, gpu);

		if(lua_pcall(m_d->L, 2, 0, 0) != 0)
		{
			ROS_ERROR("LUA error:\n%s", lua_tostring(m_d->L, -1));
			throw std::runtime_error("Could not load model");
		}
	}

	m_d->sampleDB.loadDataset(datasetPath, m_d->params);
#endif
}

void APCDenseCap::train(const std::vector<std::string>& presentObjects, const std::string& target)
{
#ifndef APC_DENSECAP_MOCK
	m_d->svm.train(m_d->sampleDB, presentObjects, target, m_d->params);
#endif
}

void APCDenseCap::trainMulti(const std::vector<std::string>& presentObjects)
{
#ifndef APC_DENSECAP_MOCK
	m_d->multiSVM.clear();
	for(auto& name : presentObjects)
	{
		m_d->multiSVM.insert(std::make_pair(name, SVMModel()));
	}

	#pragma omp parallel for
	for(size_t i = 0; i < presentObjects.size(); ++i)
	{
		m_d->multiSVM[presentObjects[i]].train(
			m_d->sampleDB, presentObjects, presentObjects[i], m_d->params
		);
	}
#endif
}

void APCDenseCap::setBinMask(const cv::Mat_<uint8_t>& binMask)
{
	int minX = std::numeric_limits<int>::max();
	int maxX = -std::numeric_limits<int>::max();
	int minY = std::numeric_limits<int>::max();
	int maxY = -std::numeric_limits<int>::max();

	for(int y = 0; y < binMask.rows; ++y)
	{
		for(int x = 0; x < binMask.cols; ++x)
		{
			if(binMask(y,x))
			{
				minX = std::min(minX, x); maxX = std::max(maxX, x);
				minY = std::min(minY, y); maxY = std::max(maxY, y);
			}
		}
	}

	m_d->binMask = binMask;
	m_d->binRect = cv::Rect(cv::Point(minX, minY), cv::Point(maxX, maxY));

	ROS_INFO_STREAM("bin rect: " << m_d->binRect);
}

apc_densecap::APCDenseCap::Result APCDenseCap::compute(const PointCloud::ConstPtr& cloud, const NormalCloud::ConstPtr& normals, const Eigen::Affine3f& cam2Shelf)
{
#ifndef APC_DENSECAP_MOCK
	APCDenseCapPrivate::ResultMatrix boxes;
	APCDenseCapPrivate::ResultMatrix features;

	m_d->callLUA(cloud, normals, cam2Shelf, &features, &boxes);

	ROS_INFO("Eigen: %ldx%ld", features.rows(), features.cols());
	Eigen::VectorXf responses = m_d->svm.scoreBatch(features);

	APCDenseCapPrivate::ResultMatrix::Index loc = -1;
	float score = -std::numeric_limits<float>::infinity();
	Rectangle bestRect;

	// convert to image space
	boxes = (1920.0/720.0)*boxes;
	boxes.col(0) = (Eigen::VectorXf::Constant(boxes.rows(), 1920 - 1) - boxes.col(0)).eval();
	boxes.col(1) = (Eigen::VectorXf::Constant(boxes.rows(), 1080 - 1) - boxes.col(1)).eval();

	for(int i = 0; i < boxes.rows(); ++i)
	{
		auto bboxData = boxes.row(i);
		Rectangle currentRect(bboxData[0], bboxData[1], bboxData[2], bboxData[3]);

		cv::Rect cvRect(currentRect.cx - currentRect.w/2, currentRect.cy - currentRect.h/2, currentRect.w, currentRect.h);

		cv::Point tl = cvRect.tl();
		cv::Point br = cvRect.br();

		tl.x = std::min(m_d->binRect.x + m_d->binRect.width, std::max(m_d->binRect.x, tl.x));
		tl.y = std::min(m_d->binRect.y + m_d->binRect.height, std::max(m_d->binRect.y, tl.y));
		br.x = std::min(m_d->binRect.x + m_d->binRect.width, std::max(m_d->binRect.x, br.x));
                br.y = std::min(m_d->binRect.y + m_d->binRect.height, std::max(m_d->binRect.y, br.y));

		cv::Rect cropped(tl, br);

		if(cropped.area() < 0.7 * cvRect.area())
			continue;

		if(responses[i] > score)
		{
			score = responses[i];
			loc = i;

			ROS_INFO_STREAM("better: " << cvRect << " => " << cropped);
			bestRect.cx = cropped.x + cropped.width/2;
			bestRect.cy = cropped.y + cropped.height/2;
			bestRect.w  = cropped.width;
			bestRect.h  = cropped.height;
		}
	}

	ROS_INFO_STREAM("max response: " << loc << " (" << boxes.row(loc) << ") with " << responses[loc]);
	ROS_INFO("cropped rect: %f %f (size %f %f)", bestRect.cx, bestRect.cy, bestRect.w, bestRect.h);

	Result result;
	result.center << bestRect.cx, bestRect.cy;
	result.size << bestRect.w, bestRect.h;
	result.response = score;

	return result;
#else
	Result result;
	return result;
#endif
}

APCDenseCap::MultiResult APCDenseCap::computeMulti(const PointCloud::ConstPtr& cloud, const NormalCloud::ConstPtr& normals, const Eigen::Affine3f& cam2Shelf)
{
#ifndef APC_DENSECAP_MOCK
	APCDenseCapPrivate::ResultMatrix boxes;
	APCDenseCapPrivate::ResultMatrix features;

	m_d->callLUA(cloud, normals, cam2Shelf, &features, &boxes);

	ROS_INFO("Eigen: %ldx%ld", features.rows(), features.cols());

	// convert to image space
	boxes = (1920.0/720.0)*boxes;
	boxes.col(0) = (Eigen::VectorXf::Constant(boxes.rows(), 1920 - 1) - boxes.col(0)).eval();
	boxes.col(1) = (Eigen::VectorXf::Constant(boxes.rows(), 1080 - 1) - boxes.col(1)).eval();

	MultiResult results;

	for(auto& pair : m_d->multiSVM)
	{
		Eigen::VectorXf responses = pair.second.scoreBatch(features);

		APCDenseCapPrivate::ResultMatrix::Index loc = -1;
		float score = -std::numeric_limits<float>::infinity();
		Rectangle bestRect;

		// Generate probability mask
		cv::Mat_<float> prob(cloud->height, cloud->width, 0.0f);
		cv::Mat_<int> count(cloud->height, cloud->width, (int)0);

		for(int i = 0; i < boxes.rows(); ++i)
		{
			auto bboxData = boxes.row(i);
			Rectangle currentRect(bboxData[0], bboxData[1], bboxData[2], bboxData[3]);

			cv::Rect cvRect(currentRect.cx - currentRect.w/2, currentRect.cy - currentRect.h/2, currentRect.w, currentRect.h);

			cv::Point tl = cvRect.tl();
			cv::Point br = cvRect.br();

			tl.x = std::min(m_d->binRect.x + m_d->binRect.width, std::max(m_d->binRect.x, tl.x));
			tl.y = std::min(m_d->binRect.y + m_d->binRect.height, std::max(m_d->binRect.y, tl.y));
			br.x = std::min(m_d->binRect.x + m_d->binRect.width, std::max(m_d->binRect.x, br.x));
			br.y = std::min(m_d->binRect.y + m_d->binRect.height, std::max(m_d->binRect.y, br.y));

			cv::Rect cropped(tl, br);

			if(cropped.width != 0 && cropped.height != 0)
			{
				double probability = 1.0 / (1.0 + exp(-4.0 * responses[i]));

				prob(cropped) += probability;
				count(cropped) += 1;
			}

			if(cropped.area() < 0.7 * cvRect.area())
				continue;

			if(responses[i] > score)
			{
				score = responses[i];
				loc = i;

// 				ROS_INFO_STREAM("better: " << cvRect << " => " << cropped);
				bestRect.cx = cropped.x + cropped.width/2;
				bestRect.cy = cropped.y + cropped.height/2;
				bestRect.w  = cropped.width;
				bestRect.h  = cropped.height;
			}
		}

		ROS_INFO_STREAM("max response: " << loc << " (" << boxes.row(loc) << ") with " << responses[loc]);
		ROS_INFO("cropped rect: %f %f (size %f %f)", bestRect.cx, bestRect.cy, bestRect.w, bestRect.h);

		cv::divide(prob, count, prob, 1.0, CV_32FC1);

		Result result;
		result.center << bestRect.cx, bestRect.cy;
		result.size << bestRect.w, bestRect.h;
		result.response = score;
		result.likelihood = prob;

		results.emplace(pair.first, result);
	}

	return results;
#else
	return MultiResult();
#endif
}

}
