// Central nodelet calling the different perception methods
// Author: Arul Selvam Periyasamy <arulselvam@uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>


/*
 * This nodelet does the following things:
 *
 * 1. Loads the pcd files for both shelf and tote, instantiates the action server and binds callbacks with subscribers
 * 2. initially all the subscribers does nothing. They are enbled by setting m_isPointCloudActive to true.
 * 3. creates two objetcs for feature computation (for shelf and tote)
 * 4. set the mesh clouds in the objects and waits for the action client
 * 5. upon the action client call arrival, it differentiates between shelf and tote request and sets the m_mode accordingly and enables the subscribers
 * 6. Point cloud subscribers invokes the rest of feature computation stuff
 * 7. All the computed features are stored in PROJECT/resource/features
 *
 */

/*
 * TODO: initial TF frames
 * TODO: publishing the ICP result and interfaces related to it
 * TODO: computing binmask
 * TODO: publishing the grasp pose
 */


#ifndef APC_PERCEPTION_PERCEPTION_NODELET_H
#define APC_PERCEPTION_PERCEPTION_NODELET_H

#include <ros/ros.h>
#include <ros/package.h>

#include <nodelet/nodelet.h>

#include <ros/subscriber.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <eigen_conversions/eigen_msg.h>

#include <apc_capture/apc_features.h>
#include <apc_capture/apc_tote_features.h>

#include <apc_capture/BoxCoordinates.h>

#include <actionlib/server/simple_action_server.h>

#include <apc_perception/ApcPerceptionAction.h>

#include <boost/filesystem.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <config_server/parameter.h>

#include <apc_densecap/apc_densecap.h>
#include <apc_6dpose/apc_6dpose.h>
#include <apc_segmentation/apc_segmentation.h>
#include <apc_objects/apc_objects.h>

#include <apc_perception/color_lut.h>

#include <thread>


namespace fs = boost::filesystem;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

namespace apc_perception
{

class PerceptionNodelet : public nodelet::Nodelet
{
public:
	PerceptionNodelet();
	virtual ~PerceptionNodelet();

private:
	//! @name general components
	//@{
		using ApcPerceptionAction = apc_perception::ApcPerceptionAction;
		using ApcPerceptionResult = apc_perception::ApcPerceptionResult;
		using PerceptionServer = actionlib::SimpleActionServer<ApcPerceptionAction>;


		void onInit () override;
		void cloudSubCallBack(const PointCloudXYZRGB::ConstPtr& cloud);

		std::vector<geometry_msgs::PoseStamped> findGraspPose(
			const PointCloudXYZRGB::ConstPtr& cloud,
			const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
			const Eigen::Affine3f& camPoints2Shelf,
			cv::Rect boundingBox, const cv::Mat_<uint8_t>& objectMask,
			const apc_objects::APCObject& info,
			float floorZ = 0.0f, bool* objectStanding = 0
		);

		std::vector<geometry_msgs::PoseStamped> findShelfGraspPose(
			const PointCloudXYZRGB::ConstPtr& cloud,
			const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
			const Eigen::Affine3f& camPoints2Shelf,
			cv::Rect boundingBox, const cv::Mat_<uint8_t>& objectMask,
			const apc_objects::APCObject& info, bool* objectStanding = 0
		);

		std::vector<geometry_msgs::PoseStamped> findToteGraspPose(
			const PointCloudXYZRGB::ConstPtr& cloud,
			const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
			const Eigen::Affine3f& camPoints2Shelf,
			cv::Rect boundingBox, const cv::Mat_<uint8_t>& objectMask,
			const apc_objects::APCObject& info, bool* objectStanding = 0
		);

		std::vector<geometry_msgs::PoseStamped> findRegistrationGraspPose(
			const PointCloudXYZRGB::ConstPtr& cloud,
			const cv::Mat_<uint8_t>& objectMask,
			const Eigen::Affine3f& cam2Container,
			const apc_objects::APCObject& info,
			float floorZ = 0.0
		);

		std::vector<geometry_msgs::PoseStamped> findShelfRegistrationGraspPose(
			const PointCloudXYZRGB::ConstPtr& cloud,
			cv::Rect boundingBox,
			const cv::Mat_<uint8_t>& objectMask,
			const Eigen::Affine3f& cam2Container,
			const apc_objects::APCObject& info
		);

		void capBoundingBox(cv::Rect& rect, int width, int height);

		void handleActionGoal();
		void handleActionCancel();

		ros::NodeHandle m_nh;
		std::unique_ptr<tf::TransformListener> m_tf;
		tf::TransformBroadcaster m_tf_broadcaster;

		ros::Subscriber m_sub_pointCloud;  // this is common for both shelf and tote

		std::unique_ptr<image_transport::ImageTransport> m_it;


		// create action server  START
		boost::shared_ptr<PerceptionServer> m_act_perceptionServer;
		apc_perception::ApcPerceptionGoal::ConstPtr m_act_goal;
		ros::Time m_actionStartTime;
		// create action server  END

		// create feature computation objects start
		apc_object_perception::ApcFeatures m_shelfFeatures;
		apc_object_perception::ApcToteFeatures m_toteFeatures;

		// RBO - CNN integration
		apc_segmentation::APCSegmentation::Result m_cnnSegmentationResult;

		// perception Model rankings
		cv::Mat m_RBOResultMask;
		cv::Mat m_CNNResultMask;
		cv::Mat m_DenseCapResultMask;
		cv::Mat m_rankedResultMask;

		// mode indicating shelf/tote
		enum Mode
		{
			MODE_SHELF,
			MODE_TOTE
		};

		Mode m_mode = MODE_SHELF;

		// Recognition modules
		enum RecognitionModule
		{
			RECOG_DENSECAP,
			RECOG_SEGMENTATION,
			RECOG_SEGMENTATION_DENSECAP,
		};

		RecognitionModule m_recognitionModule;

		bool m_registration;

		std::string m_rosPackagePath;

        apc_6dpose::APC6DPose m_6Dpose_recog;
		ros::Publisher m_pub_registration_grasps;
		ros::Publisher m_pub_registration_aligned;

		image_transport::Publisher m_pub_vis;

		//features computed
		std::map<std::string, cv::Mat > m_featureImages;

		const float m_heightNormalizer = 0.263f;
		const float m_dist2shelfNormalizer = 0.15f;

		apc_densecap::APCDenseCap m_shelf_denseCap;
		apc_densecap::APCDenseCap m_tote_denseCap;

		ros::Publisher m_pub_marker;
		ros::Publisher m_pub_pose;
		ros::Publisher m_pub_cloud;
		ros::Publisher m_pub_filteredCloud;
		ros::Publisher m_pub_filteredShelfCloud;
		ros::Publisher m_pub_transformedCloud;
		ros::Publisher m_pub_segmentCloud;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloudInContainer;

		// common features
		void extractRGBimage(const PointCloudXYZRGB::ConstPtr& cloud); // no need to handle shelf and tote separately
		void computeEdgeFeature(); // needs handling shelf and tote separately
		void computeHHA(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData);// needs handling shelf and tote separately
	//@}
	
	//! @name Shelf components
	//@{
		void camInfoCallBack(const sensor_msgs::CameraInfoConstPtr& info);

		void computeBinMask(const Eigen::Affine3f&);
		void computeShelfHeightFeatures();
		void computeDist2shelf();
		cv::Mat_<uint8_t> computeCollisionMask(
			const cv::Mat_<uint8_t>& objectMask,
			const cv::Mat_<uint8_t>& targetMask,
			uint8_t targetId
		);



		// point clouds of the meshes read from 'project/resource/***.pcd
		PointCloudXYZ::Ptr m_shelfCloudfromMesh;
		ros::Subscriber m_sub_camInfo;

		bool m_camInfoValid = false;

		Eigen::Vector2i m_boxIndices;

		//tf::StampedTransform m_tfTransShelf2Cam;
		Eigen::Affine3f m_tfCamera2Shelf;
		Eigen::Affine3f m_kinematicCamera2World;
		apc_capture::BoxCoordinates m_boxCoordinates_msg;



		apc_segmentation::APCSegmentation m_shelfSegmentation;
		apc_segmentation::APCSegmentation m_shelfBackgroundSegmentation;
	//@}

	//! @name tote components
	//@{
		void computeToteHeightFeatures();
		void computeToteMask(const Eigen::Affine3f&);

		// point clouds of the meshes read from 'project/resource/***.pcd
		PointCloudXYZ::Ptr m_toteCloudfromMesh;
		tf::StampedTransform m_tfTransCam2Tote;

		apc_segmentation::APCSegmentation m_toteSegmentation;
		ColorLUT m_lut; // for YUV classification
	//@}


	config_server::Parameter<int> m_param_toteMinPoints;


	// Saving frames
	void saveFrame(
		const boost::filesystem::path& path,
		const PointCloudXYZRGB::ConstPtr& cloud,
		const ApcPerceptionGoal::ConstPtr& goal
	);

	std::thread m_saveThread;
	unsigned int m_saveID = 0;
	boost::filesystem::path m_savePath;
	sensor_msgs::CameraInfoConstPtr m_camInfo;
	bool refineMaskforSqueaky(cv::Mat_<uint8_t>& objectMask);

	boost::filesystem::path m_currentSavePath;
};

}

#endif
