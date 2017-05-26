// Object-class segmentation for APC
// Author: Anton Milan <anton.milan@ais.uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef APC_SEGMENTATION_APC_SEGMENTATION_H
#define APC_SEGMENTATION_APC_SEGMENTATION_H

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>

namespace apc_segmentation
{

class APCSegmentationPrivate;

class APCSegmentation
{
public:
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> NormalCloud;

	struct Result
	{
		cv::Mat_<uint8_t> mask; // H x W integer mask representing per-pixel classes
		cv::Mat_<float> confidence; // H x W float representing the confidence

		cv::Mat_<float> prediction; // C x H x W per-pixel probabilities for each class
		std::vector<std::string> luaObjects; // for easy integration in Perception Nodelet

		cv::Mat_<float> objectPrediction; // C x H x W per-pixel probabilities for each _requested_ class (0 = background)
	};

	enum Mode
	{
		MODE_SHELF,
		MODE_TOTE,
		MODE_SHELF_BACKGROUND
	};

	APCSegmentation();
	virtual ~APCSegmentation();

	void initialize(Mode mode, unsigned int gpu = 2);

	void setBinMask(const cv::Mat_<uint8_t>& binMask);

	Result compute(const PointCloud::ConstPtr& cloud);
	Result compute(const PointCloud::ConstPtr& cloud, const cv::Mat_<cv::Vec3b>& hha);

	Result segmentShelf(const PointCloud::ConstPtr& cloud);
	Result segmentObjects(const PointCloud::ConstPtr& cloud, const cv::Mat3b& hha, const std::vector<std::string>& objects);

private:
	std::unique_ptr<APCSegmentationPrivate> m_d;
};

}

#endif
