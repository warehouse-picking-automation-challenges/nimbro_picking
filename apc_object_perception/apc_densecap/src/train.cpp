// Train SVM models for object recognition
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <map>
#include <memory>
#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rectangle.h"
#include "parameters.h"
#include "sample_db.h"
#include "svm.h"

using namespace apc_densecap;

namespace fs = boost::filesystem;

constexpr int NUM_PROPOSALS = 1000;
constexpr int NUM_FEATURES = 4096;
constexpr double WIDTH = 1920;
constexpr double HEIGHT = 1080;

constexpr bool DUMP_SVM = false;

void generateFineProposals(const Rectangle& box, std::vector<Rectangle>* output)
{
	double top = box.top();

	// Include the original box
	output->push_back(box);

	// Subdivide the box into K sub-boxes vertically
	for(int k = 2; k <= 3; ++k)
	{
		double h = box.h / k;

		for(int row = 0; row < k; ++row)
			output->emplace_back(box.cx, top + h/2 + row*h, box.w, h);
	}

	// Shift by 1/4 of box size in all directions
	double xshift = box.w/4;
	double yshift = box.h/4;
	for(int col = -1; col <= 1; ++col)
	{
		for(int row = -1; row <= 1; ++row)
		{
			if(row == 0 && col == 0)
				continue;

			output->emplace_back(box.cx + col * xshift, box.cy + row * yshift, box.w, box.h);
		}
	}

// 	// Subdivide the box into K sub-boxes horizontally
// 	for(int k = 2; k <= 3; ++k)
// 	{
// 		double w = box.w / k;
//
// 		for(int col = 0; col < k; ++col)
// 			output->emplace_back(left + w/2 + col*w, box.cy, w, box.h);
// 	}
//
// 	// Subdivide the box into K sub-boxes regularly
// 	for(int k = 2; k <= 2; ++k)
// 	{
// 		double w = box.w / k;
// 		double h = box.h / k;
//
// 		for(int row = 0; row < k; ++row)
// 		{
// 			for(int col = 0; col < k; ++col)
// 			{
// 				output->emplace_back(left + w/2 + col*w, top + h/2 + row*h, w, h);
// 			}
// 		}
// 	}
}

void saveProposalRequests(const fs::path& path, const std::vector<Rectangle>& requests)
{
	FILE* f = fopen(path.c_str(), "w");
	if(!f)
		throw std::runtime_error("Could not write fine proposal file");

	for(auto& req : requests)
	{
		Eigen::Vector4f boxData(
			WIDTH - 1 - req.cx, HEIGHT - 1 - req.cy,
			req.w, req.h
		);

		// Scale to CNN size
		boxData *= 720.0 / WIDTH;

		fwrite(boxData.data(), sizeof(float), 4, f);
	}

	fclose(f);
}

std::map<std::string, std::vector<Rectangle>> loadRectangles(const std::string& yamlPath)
{
	std::map<std::string, std::vector<Rectangle>> ret;

	YAML::Node doc;
	try
	{
		doc = YAML::LoadFile(yamlPath);
	}
	catch(YAML::Exception& e)
	{
		fprintf(stderr, "YAML exception on file %s: %s\n", yamlPath.c_str(), e.what());
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

struct Detection
{
	double iou;
	Eigen::VectorXf featureVector;
	Rectangle bbox;
};

void loadRawFloats(std::vector<float>* dest, const fs::path& path, bool adaptive = false)
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

static cv::Point2i rotRound(const cv::Point2d& pos)
{
	cv::Point2d rot(WIDTH - 1 - pos.x, HEIGHT - 1 - pos.y);
	return cv::Point2i(std::round(rot.x), std::round(rot.y));
}

const std::vector<cv::Scalar> COLORS{
	cv::Scalar(0, 0, 255),
	cv::Scalar(0, 255, 0),
	cv::Scalar(255, 0, 0),
	cv::Scalar(0, 255, 255),
	cv::Scalar(255, 0, 255),
	cv::Scalar(255, 255, 0),
	cv::Scalar(255, 255, 255)
};


std::vector<std::pair<float, Rectangle>> testImageSingleObject(
	const fs::path& imgPath,
	const SampleDB& trainingData, const SampleDB& testData,
	const std::vector<std::string>& presentItems,
	unsigned int k,
	const Rectangle& gtBox,
	const Parameters& params, const Eigen::ArrayXf& scale, const std::vector<Rectangle>& truth)
{
	const std::string target = presentItems[k];

	printf("Training for '%s'\n", target.c_str());

	SVMModel svm;
	svm.train(trainingData, presentItems, target, params, imgPath);


	printf("Prediction...\n");

	auto& bboxData = testData.files().at(imgPath.string())->bboxData;
	auto& featureData = testData.files().at(imgPath.string())->featureData;
	auto& hhaData = testData.files().at(imgPath.string())->hhaData;

	std::vector<std::pair<float, Rectangle>> boxes;

	FILE* svmDump = 0;
	if(DUMP_SVM)
	{
		std::stringstream ss;
		ss << "svm_" << target << ".test";
		svmDump = fopen((imgPath / ss.str()).c_str(), "w");
	}

	for(int i = 0; i < NUM_PROPOSALS; ++i)
	{
		Eigen::Vector4f bboxRaw = Eigen::Vector4f::Map(bboxData.data() + 4*i);

		bboxRaw *= WIDTH / 720.0;

		Rectangle bbox(
			WIDTH - 1 - bboxRaw[0], HEIGHT - 1 - bboxRaw[1],
			bboxRaw[2], bboxRaw[3]
		);

		// Discard bbox if it does not intersect the box enough
		if(bbox.intersection(gtBox).area() / bbox.area() < 0.85)
			continue;

		Eigen::VectorXf featureVector;
		if(params.hha)
		{
			featureVector.resize(2*NUM_FEATURES);
			featureVector <<
				Eigen::VectorXf::Map(featureData.data() + NUM_FEATURES * i, NUM_FEATURES),
				Eigen::VectorXf::Map(hhaData.data() + NUM_FEATURES * i, NUM_FEATURES)
			;
		}
		else
			featureVector = Eigen::VectorXf::Map(featureData.data() + NUM_FEATURES * i, NUM_FEATURES);

		if(DUMP_SVM)
		{
			float iou = 0.0;
			for(auto& gtRect : truth)
				iou = std::max<float>(iou, gtRect.iou(bbox));

			if(iou >= 0.5)
				fprintf(svmDump, "1");
			else
				fprintf(svmDump, "-1");

			for(int j = 0; j < featureVector.size(); ++j)
			{
				fprintf(svmDump, " %d:%f", j+1, featureVector[j]);
			}
			fprintf(svmDump, "\n");
		}

		float response = svm.score(featureVector);
		boxes.emplace_back(response, bbox);
	}

	printf("Additional bboxes...\n");

	if(params.test_proposals && fs::exists(imgPath / "rgb.png.prop.bboxout.bin"))
	{
		// Try proposal bboxes as well

		FILE* f = fopen((imgPath / "rgb.png.prop.bboxout.bin").c_str(), "r");
		fseek(f, 0, SEEK_END);
		int num = ftell(f) / sizeof(float) / 4;
		fseek(f, 0, SEEK_SET);

		printf("%d proposal bboxes available\n", num);
		std::vector<float> bboxData(num*4);

		if(fread(bboxData.data(), sizeof(float), bboxData.size(), f) != bboxData.size())
		{
			throw std::runtime_error("short read");
		}

		fclose(f);

		std::vector<float> featureData(num*NUM_FEATURES);
		loadRawFloats(&featureData, imgPath / "rgb.png.prop.feat.bin");

		for(int i = 0; i < num; ++i)
		{
			Eigen::Vector4f bboxRaw = Eigen::Vector4f::Map(bboxData.data() + 4*i);

			bboxRaw *= WIDTH / 720.0;

			Rectangle bbox(
				WIDTH - 1 - bboxRaw[0], HEIGHT - 1 - bboxRaw[1],
				bboxRaw[2], bboxRaw[3]
			);

			// Discard bbox if it does not intersect the box enough
			if(bbox.intersection(gtBox).area() / bbox.area() < 0.85)
				continue;

			Eigen::VectorXf featureVector = Eigen::VectorXf::Map(featureData.data() + NUM_FEATURES * i, NUM_FEATURES);

			if(DUMP_SVM)
			{
				float iou = 0.0;
				for(auto& gtRect : truth)
					iou = std::max<float>(iou, gtRect.iou(bbox));

				if(iou >= 0.5)
					fprintf(svmDump, "1");
				else
					fprintf(svmDump, "-1");

				for(int j = 0; j < featureVector.size(); ++j)
				{
					fprintf(svmDump, " %d:%f", j+1, featureVector[j]);
				}
				fprintf(svmDump, "\n");
			}

			float response = svm.score(featureVector);
			boxes.emplace_back(response, bbox);
		}
	}

	if(DUMP_SVM)
		fclose(svmDump);

	std::sort(boxes.begin(), boxes.end(),
		[](const std::pair<float, Rectangle>& a, const std::pair<float, Rectangle>& b)
		{ return a.first > b.first; }
	);

// 	boxes.resize(std::min<size_t>(boxes.size(), 3));

	// Calculate some interesting boxes which we want to know the features to
	std::vector<Rectangle> requests;

	for(auto& pair : boxes)
	{
		auto& box = pair.second;

		generateFineProposals(box, &requests);

		// FIXME: only use best bbox for now
		break;
	}

	if(params.visualize)
	{
		cv::Mat img = cv::imread((imgPath / "rgb.png").string());
		cv::flip(img, img, -1);

		int idx = 0;
		for(auto& rectangle : requests)
		{
			cv::rectangle(img,
				rotRound(rectangle.topLeft()), rotRound(rectangle.bottomRight()),
				COLORS[idx % COLORS.size()]
			);
			idx++;
		}

		std::stringstream ss;
		ss << "densecap_req_" << target << ".png";
		cv::imwrite((imgPath / ss.str()).string(), img);
	}

	// Write requests into data file
	{
		std::stringstream ss;
		ss << "rgb.png.densecap_req_" << target << ".bin";
		saveProposalRequests(imgPath / ss.str(), requests);
	}

	printf("Additional requests\n");

	if(params.test_request && fs::exists(imgPath / "rgb.png.req.bbox.bin"))
	{
		// Try requested bboxes as well

		FILE* f = fopen((imgPath / "rgb.png.req.bbox.bin").c_str(), "r");
		fseek(f, 0, SEEK_END);
		int num = ftell(f) / sizeof(float) / 4;
		fseek(f, 0, SEEK_SET);

		printf("%d requested bboxes available\n", num);
		std::vector<float> bboxData(num*4);

		if(fread(bboxData.data(), sizeof(float), bboxData.size(), f) != bboxData.size())
		{
			throw std::runtime_error("short read");
		}

		fclose(f);

		std::vector<float> featureData(num*NUM_FEATURES);
		loadRawFloats(&featureData, imgPath / "rgb.png.req.feat.bin");

		for(int i = 0; i < num; ++i)
		{
			Eigen::Vector4f bboxRaw = Eigen::Vector4f::Map(bboxData.data() + 4*i);

			bboxRaw *= WIDTH / 720.0;

			Rectangle bbox(
				WIDTH - 1 - bboxRaw[0], HEIGHT - 1 - bboxRaw[1],
				bboxRaw[2], bboxRaw[3]
			);

			// Discard bbox if it does not intersect the box enough
			if(bbox.intersection(gtBox).area() / bbox.area() < 0.85)
				continue;

			Eigen::VectorXf featureVector = Eigen::VectorXf::Map(featureData.data() + NUM_FEATURES * i, NUM_FEATURES);

			float response = svm.score(featureVector);
			boxes.emplace_back(response, bbox);
		}
	}

	std::sort(boxes.begin(), boxes.end(),
		[](const std::pair<float, Rectangle>& a, const std::pair<float, Rectangle>& b)
		{ return a.first > b.first; }
	);

// 	boxes.resize(std::min<size_t>(boxes.size(), 3));

#if 0
	printf("Got boxes:\n");
	for(auto& pair : boxes)
	{
		printf(" - %.3f: %f,%f,%f,%f\n", pair.first,
			pair.second.cx, pair.second.cy, pair.second.w, pair.second.h
		);
	}
#endif

	printf("Finished.\n");

	return boxes;
}

typedef std::pair<float, Rectangle> ScoredBox;
typedef std::vector<ScoredBox> ScoredBoxVector;

struct Statistics
{
	Statistics()
	 : precision(0.0)
	 , recall(0.0)
	 , count(0)
	{}

	double precision;
	double recall;
	unsigned int count;

	unsigned int failures = 0;

	ScoredBoxVector scored_boxes;
	std::vector<uint8_t> positive;

	std::size_t ground_truth_boxes = 0;
};

std::map<std::string, Statistics> testImage(const fs::path& imgPath, const SampleDB& trainingData, const SampleDB& testData, const Parameters& params, const Eigen::ArrayXf& scale)
{
	printf("Testing on image '%s'\n", imgPath.c_str());

	auto rectangles = loadRectangles((imgPath / "polygons.yaml").string());

	std::set<std::string> presentItems;
	for(auto& pair: rectangles)
	{
		printf("item: %s\n", pair.first.c_str());

		if(pair.first == "box")
			continue;

		presentItems.insert(pair.first);
	}

	const Rectangle& gtBox = rectangles["box"].at(0);

	std::vector<std::string> presentItemsVec;
	std::copy(presentItems.begin(), presentItems.end(), std::back_inserter(presentItemsVec));

	std::map<std::string, Statistics> stats;

	std::vector<std::vector<std::pair<float, Rectangle>>> allBoxes(presentItemsVec.size());

	for(unsigned int k = 0; k < presentItems.size(); ++k)
	{
		printf("Testing object '%s'\n", presentItemsVec[k].c_str());
		auto objRectangles = rectangles[presentItemsVec[k]];

		auto boxes = testImageSingleObject(imgPath, trainingData, testData, presentItemsVec, k, gtBox, params, scale, objRectangles);

		allBoxes[k] = boxes;

		Statistics stat;
		stat.ground_truth_boxes = objRectangles.size();

		std::vector<bool> used(objRectangles.size(), false);

		if(boxes.empty())
		{
			stat.precision = stat.recall = 0.0;
		}
		else
		{
			auto& pair = boxes[0];

			Rectangle groundTruth = findClosestRectangle(rectangles[presentItemsVec[k]], pair.second);

			Rectangle intersection = pair.second.intersection(groundTruth);

			stat.precision = intersection.area() / pair.second.area();
			stat.recall = intersection.area() / groundTruth.area();

			stat.scored_boxes = boxes;
			stat.positive.resize(stat.scored_boxes.size(), 0);

			for(std::size_t i = 0; i < boxes.size(); ++i)
			{
				auto& pair = boxes[i];

				if(objRectangles.empty())
					break; // no further positives possible

				auto it = findClosestRectangleIt(objRectangles.begin(), objRectangles.end(), pair.second);

				if(it == objRectangles.end())
					break;

				Rectangle groundTruth = *it;
				int idx = it - objRectangles.begin();
				if(used[idx])
					continue;

				double iou = pair.second.iou(groundTruth);

				// Is this a true positive according to VOC mAP?
				stat.positive[i] = (iou >= 0.5) ? 1 : 0;

				// We cannot take this rectangle again
				used[idx] = true;
			}
		}

		stats[presentItemsVec[k]] = stat;

		printf("done.\n");
	}

	// Build "probability" image
	for(size_t k = 0; k < presentItemsVec.size(); ++k)
	{
		auto& boxes = allBoxes[k];

		cv::Mat_<float> prob(HEIGHT, WIDTH, 0.0f);
		cv::Mat_<int> count(HEIGHT, WIDTH, (int)0);

		for(size_t i = 0; i < boxes.size(); ++i)
		{
			auto& bbox = boxes[i].second;
			cv::Point2i topLeft = bbox.topLeft();
			cv::Point2i bottomRight = bbox.bottomRight();

			topLeft.x = std::min<int>(WIDTH - 1, std::max(0, topLeft.x));
			topLeft.y = std::min<int>(HEIGHT - 1, std::max(0, topLeft.y));
			bottomRight.x = std::min<int>(WIDTH-1, std::max(topLeft.x, bottomRight.x));
			bottomRight.y = std::min<int>(HEIGHT-1, std::max(topLeft.y, bottomRight.y));

			cv::Rect rect(topLeft, bottomRight);

			double probability = 1.0 / (1.0 + exp(-4.0 * boxes[i].first));

			prob(rect) += cv::Mat_<float>(rect.height, rect.width, probability);
			count(rect) += cv::Mat_<int>(rect.height, rect.width, 1);
		}

		cv::divide(prob, count, prob, 1.0, CV_32FC1);

		double minProb;
		double maxProb;
		cv::minMaxLoc(prob, &minProb, &maxProb);

		printf("min prob: %f, max prob: %f\n", minProb, maxProb);

		cv::Mat_<uint8_t> vis;
		prob.convertTo(vis, CV_8UC1, 255.0 / maxProb);

		cv::imwrite((imgPath / ("prob_" + presentItemsVec[k] + ".png")).string(), vis);

		// Create binary output file for apc_segmentation
		{
			FILE* file = fopen((imgPath / ("prob_" + presentItemsVec[k] + ".raw")).c_str(), "w");
			fwrite(prob.data, sizeof(float), prob.rows*prob.cols, file);
			fclose(file);
		}
	}

	if(params.visualize)
	{
		cv::Mat img = cv::imread((imgPath / "rgb.png").string());

		// Rotate by 180°
		cv::flip(img, img, -1);

		for(size_t k = 0; k < presentItemsVec.size(); ++k)
		{
			auto& boxes = allBoxes[k];

			if(boxes.empty())
				continue;

			auto color = COLORS[k % COLORS.size()];

			for(size_t i = 0; i < std::min<size_t>(boxes.size(), 1); ++i)
			{
				auto& bbox = boxes[i].second;
				cv::rectangle(img, rotRound(bbox.topLeft()), rotRound(bbox.bottomRight()), color);

				{
					std::stringstream ss;
					ss << presentItemsVec[k] << " " << boxes[i].first;

					cv::putText(
						img, ss.str(), rotRound(bbox.topRight()),
						cv::FONT_HERSHEY_SIMPLEX, 1, color
					);
				}

				color = (1.0 / 1.7) * color;
			}
		}

		cv::imwrite((imgPath / "densecap.png").string(), img);
	}

	return stats;
}

void evaluate(const fs::path& dataset, const fs::path& testset, const Parameters& params)
{
	std::vector<fs::path> testsetFiles;

	if(!testset.empty())
	{
		std::ifstream input(testset.c_str());

		for(std::string line; std::getline(input, line); )
		{
			if(!line.empty())
				testsetFiles.push_back(fs::canonical(dataset / fs::path(line)));
		}
	}

	// Gather list of images
	std::vector<fs::path> jobs;
	{
		auto it = fs::recursive_directory_iterator(dataset);
		auto end = fs::recursive_directory_iterator();

		for(; it != end; ++it)
		{
			fs::path path = *it;
			if(fs::is_directory(path) && fs::exists(path / "polygons.yaml"))
			{
				path = fs::canonical(path);

				// Check if path is in testsetFiles
				auto it = std::find_if(
					testsetFiles.begin(), testsetFiles.end(),
					[&](const fs::path& p) { return p == path; }
				);

				// If not, we have a training sample
				if(it == testsetFiles.end())
					jobs.push_back(fs::canonical(path));
			}
		}
	}

	SampleDB samples;
	samples.loadDatasetFiles(params, jobs);

	SampleDB testsetStorage;
	SampleDB* testsetDB;
	if(!testset.empty())
	{
		testsetStorage.loadDatasetFiles(params, testsetFiles);
		testsetDB = &testsetStorage;
	}
	else
		testsetDB = &samples;

	printf("Training frames: %lu\n", samples.files().size());
	printf("Test set: %lu\n", testsetFiles.size());

	// Determine feature scaling
	Eigen::ArrayXf scale;
	if(params.scale)
	{
		Eigen::ArrayXf maxValues = Eigen::ArrayXf::Zero(NUM_FEATURES);

		for(auto& frame : samples.files())
		{
			for(auto& pair : frame.second->samples)
			{
				for(auto& vec : pair.second)
				{
					maxValues = maxValues.max(vec.array().abs());
				}
			}
		}

		scale = 1.0f / maxValues;
	}
	else
		scale = Eigen::ArrayXf::Constant(NUM_FEATURES, 1.0f);

	printf("\n\nEvaluating...\n");

	if(params.subsample != 1)
	{
		std::vector<fs::path> subsampled;
		for(size_t i = 0; i < testsetFiles.size(); i += params.subsample)
			subsampled.push_back(testsetFiles[i]);

		testsetFiles = std::move(subsampled);
	}

	std::map<std::string, Statistics> overall;

	#pragma omp parallel
	{
		#pragma omp for
		for(size_t i = 0; i < testsetFiles.size(); ++i)
		{
			auto stats = testImage(testsetFiles[i], samples, *testsetDB, params, scale);

			#pragma omp critical (stats)
			{
				for(auto& pair : stats)
				{
					auto& ov = overall[pair.first];
					ov.precision += pair.second.precision;
					ov.recall += pair.second.recall;
					ov.count++;

					if(pair.second.precision == 0 && pair.second.recall == 0)
						ov.failures++;

					std::copy(
						pair.second.scored_boxes.begin(), pair.second.scored_boxes.end(),
						std::back_inserter(ov.scored_boxes)
					);
					std::copy(
						pair.second.positive.begin(), pair.second.positive.end(),
						std::back_inserter(ov.positive)
					);
					ov.ground_truth_boxes += pair.second.ground_truth_boxes;
				}
			}
		}
	}

	// Normalize
	double meanPrec = 0.0;
	double meanRec = 0.0;

	for(auto& pair : overall)
	{
		pair.second.precision /= pair.second.count;
		pair.second.recall /= pair.second.count;

		meanPrec += pair.second.precision;
		meanRec += pair.second.recall;
	}

	meanPrec /= overall.size();
	meanRec /= overall.size();

	printf("\n\n\n");
	printf("Precision/recall per object:\n");
	for(auto& pair : overall)
	{
		printf(" - %35s: recall: %10.6f, precision: %10.6f (%5.3f complete failure rate)\n",
			pair.first.c_str(),
			pair.second.recall, pair.second.precision,
			float(pair.second.failures)/pair.second.count
		);
	}

	if(meanPrec + meanRec == 0.0)
	{
		printf("Precision and recall are both zero!\n");
		return;
	}

	double fScore = 2.0 * (meanPrec * meanRec) / (meanPrec + meanRec);

	printf("Mean precision: %f\n", meanPrec);
	printf("Mean recall: %f\n", meanRec);
	printf("F-Score: %f\n", fScore);

	// Calculate mAP
	double mAP = 0.0;
	{
		for(auto& pair : overall)
		{
			const std::size_t num_detections = pair.second.scored_boxes.size();

			// Sort by confidence over all images
			std::vector<std::size_t> sorted_indices(num_detections);
			std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

			std::sort(sorted_indices.begin(), sorted_indices.end(), [&](std::size_t a, std::size_t b){
				// Should a be sorted before b?
				return pair.second.scored_boxes[a].first > pair.second.scored_boxes[b].first;
			});

			Eigen::ArrayXi tp(num_detections);
			Eigen::ArrayXi fp(num_detections);

			for(std::size_t i = 0; i < num_detections; ++i)
			{
				tp[i] = pair.second.positive[sorted_indices[i]];
				fp[i] = 1 - tp[i];
			}

			// Calculate precision at every possible cut-off point
			// cumulative sum:
			for(std::size_t i = 1; i < num_detections; ++i)
			{
				tp[i] += tp[i-1];
				fp[i] += fp[i-1];
			}

			Eigen::ArrayXd recall = tp.cast<double>() / pair.second.ground_truth_boxes;
			Eigen::ArrayXd precision = tp.cast<double>() / (tp.cast<double>() + fp.cast<double>());

			// Compute max-interpolated average precision
			double ap = 0.0;
			double apn = 0.0;
			const double STEPSIZE = 0.01;
			for(std::size_t i = 0; i < 1.0 / STEPSIZE; ++i)
			{
				double t = i * STEPSIZE;

				double max_above = 0.0;
				for(std::size_t j = 0; j < num_detections; ++j)
				{
					if(recall[j] >= t)
					{
						max_above = std::max(precision[j], max_above);
					}
				}

				ap += max_above;
				apn += 1;
			}
			ap /= apn;

			printf("AP for class '%40s': %f\n", pair.first.c_str(), ap);

			mAP += ap;
		}

		mAP /= overall.size();
	}

	printf("mAP: %f\n", mAP);
}

int main(int argc, char** argv)
{
	// argument parsing
	Parameters params;

	namespace po = boost::program_options;

	po::options_description opt("Options");
	opt.add_options()
		("help", "Help message")
		("min-iou",
			po::value(&params.min_iou)->default_value(params.min_iou),
			"Minimum intersection-over-union value"
		)
		("C",
			po::value(&params.C)->default_value(params.C),
			"SVM C parameter"
		)
		("num-top-rectangles",
			po::value(&params.num_top_rectangles)->default_value(params.num_top_rectangles),
			"Maximum number of positive rectangles considered"
		)
		("bias-scale",
			po::value(&params.biasScale)->default_value(params.biasScale),
			"SVM bias scale"
		)
		("tailor",
			po::value(&params.tailor)->default_value(params.tailor),
			"Tailor SVMs to the specific case (i.e. exclude examples which cannot occur?)"
		)
		("visualize",
			po::value(&params.visualize)->default_value(params.visualize),
			"Write output pngs"
		)
		("scale",
			po::value(&params.scale)->default_value(params.scale),
			"Scale input features"
		)
		("train-request",
			po::value(&params.train_request)->default_value(params.train_request),
			"Request new bounding boxes during training (required th rerun)"
		)
		("test-request",
			po::value(&params.test_request)->default_value(params.test_request),
			"Request new bounding boxes during testing (required th rerun)"
		)
		("train-proposals",
			po::value(&params.train_proposals)->default_value(params.train_proposals),
			"Train on external object proposals as well"
		)
		("test-proposals",
			po::value(&params.test_proposals)->default_value(params.test_proposals),
			"Test on external object proposals as well"
		)
		("hha",
			po::value(&params.hha)->default_value(params.hha),
			"Use HHA features as well"
		)
		("dataset",
			po::value<std::string>()->required(),
			"Dataset to operate on"
		)
		("test-on",
			po::value<std::string>(),
			"Subset to test on"
		)
		("subsample",
			po::value(&params.subsample)->default_value(params.subsample),
			"Sub-sample testing set (1 = no subsampling)"
		)
	;

	po::positional_options_description positional;
	positional.add("dataset", 1);

	po::variables_map vm;
	po::store(
		po::command_line_parser(argc, argv).options(opt).positional(positional).run(),
		vm
	);
	po::notify(vm);

	if(vm.count("help") || !vm.count("dataset"))
	{
		std::cout << opt << std::endl;
		return 0;
	}

	std::string testSet;
	if(vm.count("test-on"))
		testSet = vm["test-on"].as<std::string>();

	evaluate(vm["dataset"].as<std::string>(), testSet, params);

	return 0;
}
