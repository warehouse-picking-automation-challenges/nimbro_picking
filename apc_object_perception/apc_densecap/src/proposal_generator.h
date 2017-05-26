// Generate object proposals from RGB-D point cloud
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PROPOSAL_GENERATOR_H
#define PROPOSAL_GENERATOR_H

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>

namespace apc_densecap
{

struct ProposalParameters
{
	float angularThreshold = 50.0f;
	float distanceThreshold = 0.002f;
	float colorThreshold = 10.0f;
	float shelfThreshold = -0.02f;
};

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> generateProposals(
	const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
	const pcl::PointCloud<pcl::Normal>::ConstPtr& normalCloud,
	const cv::Mat_<uint8_t>& mask_box,
	const ProposalParameters& params, cv::Mat_<cv::Vec3b>* vis = 0);

}

#endif

