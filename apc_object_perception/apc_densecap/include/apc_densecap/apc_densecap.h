// Main entry point for apc_densecap
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef APC_DENSECAP_H
#define APC_DENSECAP_H

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>

#include <map>

namespace apc_densecap
{

class APCDenseCapPrivate;

class APCDenseCap
{
public:
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> NormalCloud;

	struct Result
	{
		//! Position in 2D image
		Eigen::Vector2f center;

		//! Size in 2D image
		Eigen::Vector2f size;

		//! Classifier response (hint: if negative, discard!)
		float response;

		cv::Mat_<float> likelihood;
	};

	typedef std::map<std::string, Result> MultiResult;

	APCDenseCap();
	virtual ~APCDenseCap();

	void initialize(const std::string& datasetPath, int gpu = 0);

	void train(const std::vector<std::string>& presentObjects, const std::string& target);

	void setBinMask(const cv::Mat_<uint8_t>& binMask);
	Result compute(const PointCloud::ConstPtr& cloud, const NormalCloud::ConstPtr& normals, const Eigen::Affine3f& cam2Shelf);


	void trainMulti(const std::vector<std::string>& presentObjects);
	MultiResult computeMulti(const PointCloud::ConstPtr& cloud, const NormalCloud::ConstPtr& normals, const Eigen::Affine3f& cam2Shelf);

private:
	std::unique_ptr<APCDenseCapPrivate> m_d;
};

}

#endif
