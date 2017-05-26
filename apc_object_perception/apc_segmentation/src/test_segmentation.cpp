// Small test driver for the apc_segmentation library
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_segmentation/apc_segmentation.h>

#include <apc_densecap/apc_densecap.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/integral_image_normal.h>

#include <boost/filesystem.hpp>

#include <boost/make_shared.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <yaml-cpp/yaml.h>

namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
	if(argc != 2 || strcmp(argv[1], "--help") == 0)
	{
		fprintf(stderr, "Usage: test_segmentation <image path>\n");
		return 1;
	}

	fs::path imgPath(argv[1]);

	auto cloud = boost::make_shared<apc_segmentation::APCSegmentation::PointCloud>();
	if(pcl::io::loadPCDFile((imgPath / "frame.pcd").string(), *cloud) < 0)
	{
		fprintf(stderr, "Could not load PCD file\n");
		return 1;
	}

	cv::Mat_<cv::Vec3b> hha;
	hha = cv::imread((imgPath / "feature_hha.png").string());

	if(hha.empty())
	{
		fprintf(stderr, "Could not load HHA file\n");
		return 1;
	}

	cv::Mat_<uint8_t> binMask;
	binMask = cv::imread((imgPath / "mask_box.png").string(), CV_LOAD_IMAGE_GRAYSCALE);

	if(binMask.empty())
	{
		fprintf(stderr, "Could not load box mask\n");
		return 1;
	}

	binMask = 255;

	apc_segmentation::APCSegmentation segmentation;
	segmentation.initialize(apc_segmentation::APCSegmentation::MODE_TOTE);

	segmentation.setBinMask(binMask);
	
// 	printf("Shelf segmentation...\n");
// 	auto result = segmentation.segmentShelf(cloud);
// 	cv::imwrite((imgPath / "segmentation_shelf.png").string(), result.mask);
// 	printf("done.\n");

	YAML::Node doc = YAML::LoadFile((imgPath / "polygons.yaml").string());

	YAML::Node polygons = doc["polygons"];

	std::vector<std::string> objects;

	for(size_t i = 0; i < polygons.size(); ++i)
	{
		YAML::Node item = polygons[i];

		std::string name = item["name"].as<std::string>();

		if(name == "box" || name == "side_bar" || name == "ground_metal" || name == "front_bar")
			continue;

		objects.push_back(name);
	}

	auto result = segmentation.segmentObjects(cloud, hha, objects);

	printf("Segmentation finished.\n");

	cv::imwrite((imgPath / "segmentation_out.png").string(), result.mask);
	
	// write out normalized mask for display
	{
		cv::Mat_<uint8_t> vis = (255.0 / (objects.size()+1)) * result.mask;
		cv::imwrite((imgPath / "segmentation_vis.png").string(), vis);
	}

	apc_densecap::APCDenseCap densecap;
	densecap.initialize("/home/local/stud/schwarzm/apc/apc_data/tote");

	densecap.setBinMask(binMask);

	// Compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
	{
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

		ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor(0.8f);
		ne.setNormalSmoothingSize(80.0f);

		ne.setInputCloud(cloud);
		ne.compute(*normalCloud);
	}

	densecap.trainMulti(objects);
	auto densecapResult = densecap.computeMulti(cloud, normalCloud, Eigen::Affine3f::Identity());

	cv::Mat_<float> boxChannel(cloud->height, cloud->width, 1.0f);

	const float alpha = 1.0;

	for(auto& pair : densecapResult)
	{
		printf("Writing %s\n", ("prob_" + pair.first + ".png").c_str());
		cv::Mat_<uint8_t> vis;
		pair.second.likelihood.convertTo(vis, CV_8UC1, 255);
		cv::imwrite((imgPath / ("prob_" + pair.first + ".png")).string(), vis);

		double min, max;
		cv::minMaxLoc(pair.second.likelihood, &min, &max);

		pair.second.likelihood.convertTo(vis, CV_8UC1, 255.0 / max);
		cv::imwrite((imgPath / ("prob_" + pair.first + "_vis.png")).string(), vis);

		auto it = std::find(objects.begin(), objects.end(), pair.first);
		int idx = it - objects.begin();

		for(int y = 0; y < cloud->height; ++y)
		{
			for(int x = 0; x < cloud->width; ++x)
			{
				float p = (1.0f - alpha) + alpha * pair.second.likelihood(y,x) / max;

				result.objectPrediction(idx + 1, y, x) *= p;
				boxChannel(y,x) *= (1.0f - alpha) + alpha * (1.0 - pair.second.likelihood(y,x) / max);
			}
		}
	}

// 	boxChannel /= objects.size();

	cv::Mat_<uint8_t> outMask(cloud->height, cloud->width);

	// Re-normalize and make segmentation decision
	for(int y = 0; y < cloud->height; ++y)
	{
		for(int x = 0; x < cloud->width; ++x)
		{
			float sumconf = boxChannel(y,x) * result.objectPrediction(0,y,x);
			float maxconf = boxChannel(y,x) * result.objectPrediction(0,y,x);
			int maxidx = 0;

			for(size_t c = 0; c < objects.size(); ++c)
			{
				float conf = result.objectPrediction(c+1, y,x);
				sumconf += conf;

				if(conf > maxconf)
				{
					maxconf = conf;
					maxidx = c+1;
				}
			}

			outMask(y,x) = maxidx;
		}
	}

	cv::imwrite((imgPath / "segm_combined.png").string(), outMask);

	// write out normalized mask for display
	{
		cv::Mat_<uint8_t> vis = (255.0 / (objects.size()+1)) * outMask;
		cv::imwrite((imgPath / "segm_combined_vis.png").string(), vis);
	}

	return 0;
}
