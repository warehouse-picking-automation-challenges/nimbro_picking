// Nodelet for fusing (i.e. projecting together) two depth streams
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_REALSENSE_DEPTH_FUSION_H
#define NIMBRO_REALSENSE_DEPTH_FUSION_H

#include <nodelet/nodelet.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

#include <Eigen/Core>

#include <config_server/parameter.h>

#include <depth_filler/domain_transform.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mutex>
#include <thread>
#include <condition_variable>

namespace nimbro_realsense
{

class DepthFusion : public nodelet::Nodelet
{
public:
	DepthFusion();
	virtual ~DepthFusion();

	virtual void onInit() override;
private:
	typedef pcl::PointXYZRGB Point;
	typedef pcl::PointCloud<Point> Cloud;

	struct Measurement
	{
		enum Source
		{
			SOURCE_STEREO,
			SOURCE_UPPER_RGBD,
			SOURCE_LOWER_RGBD
		};

		Measurement(Source source, float depth, float weight)
		 : source(source)
		 , depth(depth)
		 , weight(weight)
		{}

		Source source;
		float depth;
		float weight;
	};

	void checkSubscribers();

	void debugCloud(const std::string& name, const sensor_msgs::PointCloud2ConstPtr& msg);

	void handleClouds(
		const sensor_msgs::PointCloud2ConstPtr& upper,
		const sensor_msgs::PointCloud2ConstPtr& lower,
		const sensor_msgs::PointCloud2ConstPtr& stereo
	);

	void handleCameraInfoUpper(const sensor_msgs::CameraInfoConstPtr& info);
	void handleCameraInfoLower(const sensor_msgs::CameraInfoConstPtr& info);
	void handleCameraInfoStereo(const sensor_msgs::CameraInfoConstPtr& info);

	void process(const Cloud& cloud_upper, const Cloud& cloud_lower, const Cloud& cloud_stereo);

	bool calibrateDepthMaps(const cv::Mat_<float>& source, const cv::Mat_<float>& target, Eigen::Vector2f* result);

	void processThread();

	std::mutex m_mutex;
	std::condition_variable m_cond;

	std::thread m_processThread;
	bool m_shouldExit = false;

	bool m_active = false;
	ros::Time m_subscribeTime;

	tf::TransformListener m_tf;

	typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSubscriber;

	CloudSubscriber m_sub_cloud_upper;
	CloudSubscriber m_sub_cloud_lower;
	CloudSubscriber m_sub_cloud_stereoInUpper;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> CloudSyncPolicy;
	typedef message_filters::Synchronizer<CloudSyncPolicy> CloudSynchronizer;

	std::unique_ptr<CloudSynchronizer> m_sync_cloud;

	ros::Subscriber m_sub_camInfo_upper;
	ros::Subscriber m_sub_camInfo_lower;
	ros::Subscriber m_sub_camInfo_stereo;

	ros::Publisher m_pub_cloud;

	ros::Publisher m_pub_correctedStereo;

	sensor_msgs::PointCloud2ConstPtr m_cloud_upper;
	sensor_msgs::PointCloud2ConstPtr m_cloud_lower;
	sensor_msgs::PointCloud2ConstPtr m_cloud_stereo;

	image_geometry::PinholeCameraModel m_model_upper;
	image_geometry::PinholeCameraModel m_model_lower;
	image_geometry::PinholeCameraModel m_model_stereo;

	bool m_calibrate = true;
	float m_calibScale = 1.0f;
	float m_calibOffset = 0.0f;

	float m_lowerScale = 1.0f;
	float m_lowerOffset = 0.0f;

	Eigen::Matrix3f m_R = Eigen::Matrix3f::Identity();
	Eigen::Vector3f m_T = Eigen::Vector3f::Zero();

	bool m_stereoMapsInitialized = false;
	cv::Mat_<float> m_stereoMapX;
	cv::Mat_<float> m_stereoMapY;

	config_server::Parameter<bool> m_param_fill_input;
	depth_filler::DomainTransformFiller m_filler;

	config_server::Parameter<float> m_param_weight_upper;
	config_server::Parameter<float> m_param_weight_lower;
	config_server::Parameter<float> m_param_weight_stereo;

	config_server::Parameter<float> m_param_trust_stereo_threshold;
};

}

#endif
