#ifndef APC_6DPOSE_H
#define APC_6DPOSE_H

#include <memory>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/core/core.hpp>


#include <iostream>
#include <sstream>


namespace apc_6dpose
{

void readGraspingPositions(const std::string filename, pcl::PointCloud<pcl::PointXYZ> &grasp_points, pcl::PointCloud<pcl::PointXYZ> &grasp_directions);
class APC6DPosePrivate;

class APC6DPose
{
public:
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    typedef pcl::PointXYZRGB PointT;


	struct Result
	{
        Eigen::Matrix4f objectPose;
        Eigen::Vector4f objectCenter;
        pcl::PointCloud<pcl::PointXYZ> grasping_points;
        pcl::PointCloud<pcl::PointXYZ> grasping_directions;
        pcl::PointCloud<pcl::PointXYZRGB> aligned_model;
        float distance;
        float confidence;
	};

	APC6DPose(int argc, char* argv[]);
	APC6DPose();

	virtual ~APC6DPose();

	void setBinMask(const cv::Mat_<uint8_t>& binMask);

	/**
	 * @param cloud Input point cloud (from camera perspective)
	 * @param object Object name to be registered
	 * @param hint Point in the point cloud frame where the object center might be
	 **/
    APC6DPose::Result compute(const PointCloud::ConstPtr& cloud, const std::string& object, const Eigen::Vector3f& hint, const cv::Mat_<uint8_t> &mask);
    APC6DPose::Result compute(const PointCloud::ConstPtr& cloud);
	void initialize(int argc, char* argv[]);
    void initialize(std::string models_folder, std::string grasp_file);
    void setUseVisibility(bool use_visibility_reasoning );
    void setICPType(int icp_type);
    void setPerformDilate(bool perform_dilate);
    void setUseNaiveApproach(bool use_naive_approach);


private:
    std::unique_ptr<APC6DPosePrivate> m_p;

};

}

#endif
