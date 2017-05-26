// Holds training samples for SVM training / testing
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "sample_db.h"

#include <yaml-cpp/yaml.h>

#include <opencv2/highgui/highgui.hpp>

namespace fs = boost::filesystem;

// FIXME: Can we get rid of these?
constexpr int NUM_PROPOSALS = 1000;
constexpr int NUM_FEATURES = 4096;
constexpr double WIDTH = 1920;
constexpr double HEIGHT = 1080;

namespace apc_densecap
{

std::map<std::string, std::vector<Rectangle>> apc_densecap::SampleDB::loadRectangles(
	const boost::filesystem::path& yamlPath)
{
	std::map<std::string, std::vector<Rectangle>> ret;

	YAML::Node doc;
	try
	{
		doc = YAML::LoadFile(yamlPath.string());
	}
	catch(YAML::Exception& e)
	{
		fprintf(stderr, "YAML exception on file '%s': %s\n", yamlPath.c_str(), e.what());
		throw;
	}

	YAML::Node polygons = doc["polygons"];

	for(size_t i = 0; i < polygons.size(); ++i)
	{
		YAML::Node item = polygons[i];

		YAML::Node points = item["points"];
		if(points.size() == 0)
			continue;

		std::string name = item["name"].as<std::string>();
		if(name == "ground_metal" || name == "front_bar" || name == "side_bar")
			continue;

		int min_x, max_x, min_y, max_y;
		min_x = max_x = points[0][0].as<int>();
		min_y = max_y = points[0][1].as<int>();

		for(size_t j = 1; j < points.size(); ++j)
		{
			int x = points[j][0].as<int>();
			int y = points[j][1].as<int>();
			min_x = std::min(x, min_x);
			max_x = std::max(x, max_x);
			min_y = std::min(y, min_y);
			max_y = std::max(y, max_y);
		}

		auto it = ret.find(name);
		if(it == ret.end())
			it = ret.emplace(name, std::vector<Rectangle>()).first;

		it->second.emplace_back(
			0.5*(max_x + min_x), 0.5*(max_y + min_y),
			max_x - min_x, max_y - min_y
		);
	}

	// Merge intersecting bboxes
	// FIXME: Revise this!
	for(auto& pair : ret)
	{
		for(ssize_t j = pair.second.size()-1; j >= 0; --j)
		{
			for(ssize_t k = j-1; k >= 0; --k)
			{
				if(pair.second[j].intersection(pair.second[k]).area() > 0.0)
				{
// 					printf("Merging bbox for %s\n", pair.first.c_str());
					pair.second[k] = pair.second[k].unionRect(pair.second[j]);
					pair.second.erase(pair.second.begin() + j);
					break;
				}
			}
		}
	}

	return ret;
}

void SampleDB::loadRawFloats(std::vector<float>* dest, const fs::path& path, bool adaptive)
{
	FILE* f = fopen(path.c_str(), "r");
	if(!f)
	{
		std::stringstream ss;
		ss << "Could not open data file " << path.string();
		throw std::runtime_error(ss.str());
	}

	if(adaptive)
	{
		fseek(f, 0, SEEK_END);
		dest->resize(ftell(f) / sizeof(float));
		fseek(f, 0, SEEK_SET);
	}

	size_t cnt = fread(dest->data(), sizeof(float), dest->size(), f);
	if(cnt != dest->size())
	{
		std::stringstream ss;
		ss << "Short/failed read from data file " << path.string();
		throw std::runtime_error(ss.str());
	}

	fclose(f);
}

struct Detection
{
	double iou;
	Eigen::VectorXf featureVector;
	Rectangle bbox;
};

static cv::Point2i rotRound(const cv::Point2d& pos)
{
	cv::Point2d rot(WIDTH - 1 - pos.x, HEIGHT - 1 - pos.y);
	return cv::Point2i(std::round(rot.x), std::round(rot.y));
}

static const std::vector<cv::Scalar> COLORS{
	cv::Scalar(0, 0, 255),
	cv::Scalar(0, 255, 0),
	cv::Scalar(255, 0, 0),
	cv::Scalar(0, 255, 255),
	cv::Scalar(255, 0, 255),
	cv::Scalar(255, 255, 0),
	cv::Scalar(255, 255, 255)
};

SampleDB::TrainingFrame::Ptr SampleDB::loadImage(
	const boost::filesystem::path& path, const apc_densecap::Parameters& params)
{
	auto rectangles = loadRectangles((path / "polygons.yaml").c_str());

	TrainingFrame::Ptr frame(new TrainingFrame);

	frame->featureData.resize(NUM_PROPOSALS * NUM_FEATURES);
	loadRawFloats(&frame->featureData, path / "rgb.png.feat.bin");

	frame->bboxData.resize(NUM_PROPOSALS * 4);
	loadRawFloats(&frame->bboxData, path / "rgb.png.bbox.bin");

	if(params.hha)
	{
		frame->hhaData.resize(NUM_PROPOSALS * NUM_FEATURES);
		loadRawFloats(&frame->hhaData, path / "rgb.png.hha.feat.bin");
	}

	std::map<std::string, std::vector<Detection>> detections;

	for(int k = 0; k < NUM_PROPOSALS; ++k)
	{
		Eigen::Vector4f bboxValues = Eigen::Vector4f::Map(frame->bboxData.data() + k*4);

		bboxValues *= WIDTH / 720.0;
		Rectangle bbox = Rectangle(
			WIDTH - 1 - bboxValues[0],
			HEIGHT - 1 - bboxValues[1],
			bboxValues[2], bboxValues[3]
		);

		Eigen::VectorXf featureVector;

		if(params.hha)
		{
			featureVector.resize(NUM_FEATURES*2);
			featureVector <<
				Eigen::VectorXf::Map(frame->featureData.data() + k*NUM_FEATURES, NUM_FEATURES),
				Eigen::VectorXf::Map(frame->hhaData.data() + k*NUM_FEATURES, NUM_FEATURES)
			;
		}
		else
			featureVector = Eigen::VectorXf::Map(frame->featureData.data() + k*NUM_FEATURES, NUM_FEATURES);

		bool found_match = false;
		for(auto& pair : rectangles)
		{
			if(pair.first == "box")
				continue;

			for(auto& rect : pair.second)
			{
				double iou = bbox.iou(rect);
				if(iou >= params.min_iou)
				{
					// Positive example!
					Detection det;
					det.iou = iou;
					det.featureVector = featureVector;
					det.bbox = bbox;

					detections[pair.first].push_back(std::move(det));
					found_match = true;
				}
			}
		}

		if(!found_match)
			frame->samples["negative"].push_back(featureVector);
	}

	// Analyze external object proposals if present
	if(params.train_proposals && fs::exists(path / "rgb.png.prop.bboxout.bin"))
	{
		std::vector<float> bboxData;
		loadRawFloats(&bboxData, path / "rgb.png.prop.bboxout.bin", true);

		std::vector<float> featureData(bboxData.size() / 4 * NUM_FEATURES);
		loadRawFloats(&featureData, path / "rgb.png.prop.feat.bin");

		for(unsigned int k = 0; k < bboxData.size()/4; ++k)
		{
			Eigen::Vector4f bboxValues = Eigen::Vector4f::Map(bboxData.data() + k*4);

			bboxValues *= WIDTH / 720.0;
			Rectangle bbox = Rectangle(
				WIDTH - 1 - bboxValues[0],
				HEIGHT - 1 - bboxValues[1],
				bboxValues[2], bboxValues[3]
			);

			Eigen::VectorXf featureVector = Eigen::VectorXf::Map(featureData.data() + k*NUM_FEATURES, NUM_FEATURES);

			bool found_match = false;
			for(auto& pair : rectangles)
			{
				if(pair.first == "box")
					continue;

				for(auto& rect : pair.second)
				{
					double iou = bbox.iou(rect);
					if(iou >= params.min_iou)
					{
						// Positive example!
						Detection det;
						det.iou = iou;
						det.featureVector = featureVector;
						det.bbox = bbox;

						detections[pair.first].push_back(std::move(det));
						found_match = true;
					}
				}
			}

			if(!found_match)
				frame->samples["negative"].push_back(featureVector);
		}
	}

	for(auto& pair : detections)
	{
		// Sort descending by iou
		std::sort(pair.second.begin(), pair.second.end(),
			[](const Detection& a, const Detection& b){ return a.iou > b.iou; }
		);

		unsigned int num_top = std::min<unsigned int>(
			pair.second.size(), params.num_top_rectangles
		);
		auto& dsamples = frame->samples[pair.first];

		for(unsigned int i = 0; i < num_top; ++i)
		{
			dsamples.push_back(pair.second[i].featureVector);
		}
	}

	if(params.visualize)
	{
		cv::Mat img = cv::imread((path / "rgb.png").string());
		cv::flip(img, img, -1);

		for(auto& pair : detections)
		{
			cv::Mat objImg;
			img.copyTo(objImg);

			unsigned int num = std::min<size_t>(pair.second.size(), params.num_top_rectangles);
			for(unsigned int i = 0; i < num; ++i)
			{
				auto& bbox = pair.second[i].bbox;
				cv::rectangle(objImg,
					rotRound(bbox.topLeft()), rotRound(bbox.bottomRight()),
					COLORS[i % COLORS.size()]
				);
			}

			std::stringstream ss;
			ss << "densecap_training_" << pair.first << ".png";
			cv::imwrite((path / ss.str()).string(), objImg);
		}
	}

	return frame;
}

void SampleDB::loadDatasetFiles(const Parameters& params, const std::vector<fs::path>& files)
{
	// Load training data
	printf("Loading training data...\n");

	printf("000/%03lu", files.size());
	fflush(stdout);

	{
		#pragma omp parallel for
		for(size_t i = 0; i < files.size(); ++i)
		{
			auto data = SampleDB::loadImage(files[i], params);

			#pragma omp critical (data)
			{
				m_data[files[i].string()] = data;

				printf("\r%03lu/%03lu", m_data.size(), files.size());
				fflush(stdout);
			}
		}
	}
	printf("\n");

	// Training data statistics
	printf("Training samples per object:\n");
	{
		std::map<std::string, std::pair<unsigned int, unsigned int>> samples;
		for(auto& file : m_data)
		{
			for(auto& object : file.second->samples)
			{
				auto it = samples.insert(std::make_pair(object.first, std::make_pair(0u, 0u))).first;
				it->second.first += object.second.size();
				it->second.second += 1;
			}
		}

		for(auto& object : samples)
		{
			printf(" - %35s: %8u (%2u images)\n", object.first.c_str(), object.second.first, object.second.second);
		}
	}
}

void SampleDB::loadDataset(const boost::filesystem::path& path, const Parameters& params)
{
	// Gather list of images
	std::vector<fs::path> jobs;
	{
		auto it = fs::recursive_directory_iterator(path);
		auto end = fs::recursive_directory_iterator();

		for(; it != end; ++it)
		{
			fs::path path = *it;
			if(fs::is_directory(path) && fs::exists(path / "polygons.yaml"))
				jobs.push_back(fs::canonical(path));
		}
	}

	loadDatasetFiles(params, jobs);
}

}
