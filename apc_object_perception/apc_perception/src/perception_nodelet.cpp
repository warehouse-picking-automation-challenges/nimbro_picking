// Central nodelet calling the different perception methods
// Author: Arul Selvam Periyasamy <arulselvam@uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_perception/perception_nodelet.h>

#include <apc_objects/apc_objects.h>

#include <apc_capture/BoxCoordinates.h>

#include <apc_shelf_model/dimensions.h>

#include <pluginlib/class_list_macros.h>

#include <tf_conversions/tf_eigen.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <camera_calibration_parsers/parse_ini.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/features/integral_image_normal.h>

#include <visualization_msgs/Marker.h>
#include <camera_calibration_parsers/parse.h>

#include <memory>
#include <future>

#include "contrib/connectedcomponents.h"
#include "contrib/husl/husl.h"


static const std::vector<cv::Scalar> COLORS{
	cv::Scalar(0, 0, 255),
	cv::Scalar(0, 255, 0),
	cv::Scalar(255, 0, 0),
	cv::Scalar(0, 255, 255),
	cv::Scalar(255, 0, 255),
	cv::Scalar(255, 255, 0),
	cv::Scalar(255, 255, 255)
};

static const float tote_X = 0.30f, tote_Y = 0.18f, tote_Z = 0.2f;

static constexpr float collisionMaskMeterPerPixel = 0.002f; // meter

static std::vector<cv::Vec3b> huslColors(size_t n)
{
	std::vector<cv::Vec3b> ret;

	// Sample colors from the HUSL space
	for(size_t i = 0; i < n; ++i)
	{
		float hue = i * 360 / n;
		float sat = (i % 2) ? 80 : 100;
		float lum = (i % 2) ? 40 : 60;

		float r, g, b;
		HUSLtoRGB(&r, &g, &b, hue, sat, lum);

		r *= 255.0;
		g *= 255.0;
		b *= 255.0;

		ret.emplace_back(
			cv::saturate_cast<uint8_t>(b),
			cv::saturate_cast<uint8_t>(g),
			cv::saturate_cast<uint8_t>(r)
		);
	}

	return ret;
}

namespace
{

	class ScopeTimer
	{
	public:
		ScopeTimer()
		{
			m_startTime = ros::WallTime::now();
		}

		~ScopeTimer()
		{
			ros::WallTime time = m_startTime;

			ROS_INFO("timings:");

			for(auto& pair : m_checkpoints)
			{
				ROS_INFO(" - %20s: %10.4fs", pair.first.c_str(), (pair.second - time).toSec());
				time = pair.second;
			}

			ROS_INFO("total time: %10.4fs", (ros::WallTime::now() - m_startTime).toSec());
		}

		void addCheckpoint(const std::string& label)
		{
			m_checkpoints.emplace_back(label, ros::WallTime::now());
		}
	private:
		ros::WallTime m_startTime;
		std::vector<std::pair<std::string, ros::WallTime>> m_checkpoints;
	};

}

namespace apc_perception
{

PerceptionNodelet::PerceptionNodelet()
 : m_param_toteMinPoints("/perception/tote/minPoints", 0, 1, 10000, 400)
{
}

PerceptionNodelet::~PerceptionNodelet()
{
	if(m_saveThread.joinable())
		m_saveThread.join();
}

void PerceptionNodelet::capBoundingBox(cv::Rect& rect, int width, int height)
{
	rect.x = std::max(0, std::min(width-1, rect.x));
	rect.y = std::max(0, std::min(height-1, rect.y));

	rect.width = std::min(rect.width, width - rect.x);
	rect.height = std::min(rect.height, height - rect.y);
}

void PerceptionNodelet::onInit()
{
	m_nh = getPrivateNodeHandle();

	std::string modeString;
	if(!m_nh.getParam("mode", modeString))
		throw std::runtime_error("perception nodelet needs mode parameter");

	if(modeString == "shelf")
		m_mode = MODE_SHELF;
	else if(modeString == "tote")
		m_mode = MODE_TOTE;
	else
		throw std::runtime_error("unknown mode parameter");

	{
		char buf[100];
		auto t = time(NULL);
		auto tmp = localtime(&t);
		strftime(buf, sizeof(buf), "run_%Y%m%d_%H%M", tmp);

		m_savePath = boost::filesystem::path("/tmp") / buf;
	}

	m_sub_camInfo = m_nh.subscribe("camera_info",1,&PerceptionNodelet::camInfoCallBack,this);

	// load colors_apc.lut file
	if(!m_lut.loadFromFile( ros::package::getPath("apc_perception")+"/resource/colors_apc.lut"))
	{
		NODELET_FATAL("Could not load LUT file");
		throw std::runtime_error("Could not load LUT file");
	}

	m_pub_marker = m_nh.advertise<visualization_msgs::Marker>("grasp_pose", 1, true);
	m_pub_pose = m_nh.advertise<geometry_msgs::PoseStamped>("grasp_pose_pose", 1, true);

	m_tf.reset(new tf::TransformListener(m_nh, ros::Duration(30.0)));

	m_it.reset(new image_transport::ImageTransport(m_nh));
	m_pub_vis = m_it->advertise("vis", 10);
	m_pub_cloud = m_nh.advertise<PointCloudXYZ>("used_cloud", 1, true);
	m_pub_filteredCloud = m_nh.advertise<PointCloudXYZ>("filtered_cloud", 1, true);
	m_pub_filteredShelfCloud = m_nh.advertise<PointCloudXYZ>("filtered_shelf_cloud", 1, true);
	m_pub_transformedCloud = m_nh.advertise<PointCloudXYZ>("transformed_cloud", 1, true);
	m_pub_segmentCloud = m_nh.advertise<PointCloudXYZRGB>("segment_cloud", 1, true);

	m_pub_registration_grasps = m_nh.advertise<visualization_msgs::Marker>("registration/grasps", 1, true);
	m_pub_registration_aligned = m_nh.advertise<PointCloudXYZRGB>("registration/aligned", 1, true);

	// load the shelf  and tote point cloud and set them in the feature computing objects
	{
		m_shelfCloudfromMesh.reset(new pcl::PointCloud<pcl::PointXYZ>());
		m_toteCloudfromMesh.reset(new pcl::PointCloud<pcl::PointXYZ>());
		m_rosPackagePath = ros::package::getPath("apc_capture");

		// shelf
		std::string MeshPath = m_rosPackagePath + "/resource/pcd/shelf_withbackface.pcd";
		if (pcl::io::loadPCDFile(MeshPath, *m_shelfCloudfromMesh) < 0)
		{
			ROS_ERROR_STREAM("Could not load shelf mesh");
			throw std::runtime_error("Could not load shelf mesh");
		}
		// Translate to apc_objects::distRobot2Shelf (FIXME: This is a magic number)
		Eigen::Affine3f translation(Eigen::Translation3f(apc_objects::distRobot2Shelf,0,0));
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*m_shelfCloudfromMesh,*cloudTemp,translation.matrix());
		m_shelfCloudfromMesh=cloudTemp;

		// tote
		MeshPath = m_rosPackagePath + "/resource/pcd/tote_face_removed.pcd";
		if (pcl::io::loadPCDFile(MeshPath, *m_toteCloudfromMesh) < 0)
		{
			ROS_ERROR_STREAM("Could not load tote mesh");
			throw std::runtime_error("Could not load tote mesh");
		}

		m_shelfFeatures.setMeshCloud(m_shelfCloudfromMesh);
		m_toteFeatures.setMeshCloud(m_toteCloudfromMesh);
	}
	ROS_INFO_STREAM(" Mesh loaded");

	std::string recognitionModule;
	m_nh.param("recognition_module", recognitionModule, std::string("segmentation_densecap"));

	if(recognitionModule == "densecap")
		m_recognitionModule = RECOG_DENSECAP;
	else if(recognitionModule == "segmentation")
		m_recognitionModule = RECOG_SEGMENTATION;
	else if(recognitionModule == "segmentation_densecap")
		m_recognitionModule = RECOG_SEGMENTATION_DENSECAP;
	else
	{
		NODELET_ERROR("Invalid recognition module '%s'", recognitionModule.c_str());
		throw std::runtime_error("invalid recognition module");
	}

	m_nh.param("registration", m_registration, false);

	int segmentationGPU;
	m_nh.param("segmentation_gpu", segmentationGPU, 2);

	// Recognition module initialization
	switch(m_recognitionModule)
	{
		case RECOG_DENSECAP:
		case RECOG_SEGMENTATION_DENSECAP:
		{
			std::string datasetPath;
			if(!m_nh.getParam("dataset_path", datasetPath))
			{
				NODELET_ERROR("densecap module needs dataset_path parameter");
				throw std::runtime_error("densecap module needs dataset_path parameter");
			}

			int gpu;
			m_nh.param("densecap_gpu", gpu, 0);

			if(m_mode == MODE_SHELF)
			{
				m_shelf_denseCap.initialize(datasetPath + "/shelf", gpu);
				m_shelfSegmentation.initialize(apc_segmentation::APCSegmentation::MODE_SHELF, segmentationGPU);
			}
			else
			{
				m_tote_denseCap.initialize(datasetPath + "/tote", gpu);
				m_toteSegmentation.initialize(apc_segmentation::APCSegmentation::MODE_TOTE, segmentationGPU);
			}
			break;
		}
		case RECOG_SEGMENTATION:
			if(m_mode == MODE_SHELF)
				m_shelfSegmentation.initialize(apc_segmentation::APCSegmentation::MODE_SHELF, segmentationGPU);
			else
				m_toteSegmentation.initialize(apc_segmentation::APCSegmentation::MODE_TOTE, segmentationGPU);
			break;
	}

	if(m_mode == MODE_SHELF)
		m_shelfBackgroundSegmentation.initialize(apc_segmentation::APCSegmentation::MODE_SHELF_BACKGROUND, segmentationGPU);

	// initialize the action server
	m_act_perceptionServer.reset(new PerceptionServer(m_nh, "perception", false));

	// bind the call back with action server
	m_act_perceptionServer->registerGoalCallback(
		boost::bind(&PerceptionNodelet::handleActionGoal, this)
	);
	m_act_perceptionServer->registerPreemptCallback(
		boost::bind(&PerceptionNodelet::handleActionCancel, this)
	);

	// start the server
	m_act_perceptionServer->start();
}

void PerceptionNodelet::saveFrame(const boost::filesystem::path& path, const PointCloudXYZRGB::ConstPtr& cloud, const ApcPerceptionGoal::ConstPtr& goal)
{
	pcl::io::savePCDFileBinaryCompressed((path / "frame.pcd").string(), *cloud);

	cv::Mat_<cv::Vec3b> rgb(cloud->height, cloud->width);
	for(unsigned int y = 0; y < cloud->height; ++y)
	{
		for(unsigned int x = 0; x < cloud->width; ++x)
		{
			auto& p = (*cloud)(x,y);
			rgb(y,x) = cv::Vec3b(p.b, p.g, p.r);
		}
	}

	cv::imwrite((path / "rgb.png").string(), rgb);

	camera_calibration_parsers::writeCalibration((path / "camera.ini").string(), "camera", *m_camInfo);

	{
		std::ofstream context((path / "box_index.txt").string());

		if(goal->isShelf)
			context << goal->box_row << ' ' << goal->box_col << '\n';
		else
			context << "10 10\n";
	}
}

void PerceptionNodelet::handleActionGoal()
{
	if(!m_act_perceptionServer->isNewGoalAvailable())
		return;

	m_actionStartTime = ros::Time::now();
	m_act_goal = m_act_perceptionServer->acceptNewGoal();

	m_sub_pointCloud = m_nh.subscribe("cloud", 1, &PerceptionNodelet::cloudSubCallBack, this);
	NODELET_INFO("Subscribed to %s", m_sub_pointCloud.getTopic().c_str());

	m_shelfFeatures.setBoxIndices(Eigen::Vector2i(m_act_goal->box_row, m_act_goal->box_col));

	auto requestedMode = m_act_goal->isShelf ? MODE_SHELF : MODE_TOTE;
	if(requestedMode != m_mode)
	{
		NODELET_ERROR("Nodelet has been started with mode %d, but action wants %d", m_mode, requestedMode);
		m_act_perceptionServer->setAborted(ApcPerceptionResult());
		return;
	}

	ROS_INFO_STREAM("new action : perception for " << ((m_mode == MODE_SHELF) ? "shelf" : "tote") );

	switch(m_recognitionModule)
	{
		case RECOG_DENSECAP:
			if(m_mode == MODE_SHELF)
				m_shelf_denseCap.train(m_act_goal->candidate_objects, m_act_goal->desired_object);
			else
				m_tote_denseCap.trainMulti(m_act_goal->candidate_objects);
			break;
		case RECOG_SEGMENTATION_DENSECAP:
			if(m_mode == MODE_SHELF)
				m_shelf_denseCap.trainMulti(m_act_goal->candidate_objects);
			else
				m_tote_denseCap.trainMulti(m_act_goal->candidate_objects);
			break;
		case RECOG_SEGMENTATION:
			break;
	}

	// Delete grasp pose marker
	{
		visualization_msgs::Marker marker;
		marker.action = marker.DELETE;
		m_pub_marker.publish(marker);

		geometry_msgs::PoseStamped pose;
		m_pub_pose.publish(pose);

		PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
		cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
		cloud->header.frame_id = "world";
		m_pub_registration_aligned.publish(cloud);
	}
}

void PerceptionNodelet::handleActionCancel()
{
	m_act_goal.reset();

	NODELET_INFO("Canceled, shutting down subscriber...");
	m_sub_pointCloud.shutdown();

	apc_perception::ApcPerceptionResult result;
	m_act_perceptionServer->setPreempted(result);
}

void PerceptionNodelet::camInfoCallBack(const sensor_msgs::CameraInfoConstPtr& msg)
{
	m_camInfo = msg;
	m_camInfoValid = true;
	m_shelfFeatures.setCameraModel(msg);
	m_toteFeatures.setCameraModel(msg);
}

void PerceptionNodelet::cloudSubCallBack(const PointCloudXYZRGB::ConstPtr& cloud)
{
	if(!m_act_goal)
		return;

	if(!m_camInfoValid)
		return;

	if(pcl_conversions::fromPCL(cloud->header.stamp) < m_actionStartTime)
		return;

	ScopeTimer timings;

	ApcPerceptionResult perceptionResult;

	m_pub_cloud.publish(cloud);

	{
		if(m_saveThread.joinable())
			m_saveThread.join();

		char buf[100];
		snprintf(buf, sizeof(buf), "image_%03d", m_saveID++);

		m_currentSavePath = m_savePath / buf;
		boost::filesystem::create_directories(m_currentSavePath);

		// Save PCD files in background (takes a while)
		m_saveThread = std::thread(std::bind(&PerceptionNodelet::saveFrame, this, m_currentSavePath, cloud, m_act_goal));
	}

	// tf lookup sensorframe to world frame
	// FIXME: tf::MessageFilter
	if(!m_tf->waitForTransform("world", cloud->header.frame_id,  pcl_conversions::fromPCL(cloud->header.stamp), ros::Duration(2.0)))
	{
		NODELET_ERROR(" The transform wait from cloud to world timeout ");
		return;
	}
	//pcl::io::savePCDFile("/home/arul/arul/apc/bags/NewDS/frame.pcd",*cloud);
	tf::StampedTransform tfTrans;
	try
	{
		m_tf->lookupTransform("world", cloud->header.frame_id,
			pcl_conversions::fromPCL(cloud->header.stamp), tfTrans
		);
	}
	catch(tf::TransformException& e)
	{
		NODELET_ERROR("Could not get input transform: %s", e.what());
		return;
	}

	// transform point cloud to the world coordinate
	Eigen::Affine3d transformSensorInWorldFrame;
	tf::transformTFToEigen(tfTrans, transformSensorInWorldFrame);
	m_kinematicCamera2World = transformSensorInWorldFrame.cast<float>();

	timings.addCheckpoint("transformed");

	if(m_mode == MODE_SHELF)
	{
		const apc_objects::APCObject* objectInfo;
		{
			auto it = apc_objects::objects().find(m_act_goal->desired_object);
			if(it == apc_objects::objects().end())
			{
				NODELET_ERROR("Specified unknown object '%s' in perception action.",
					m_act_goal->desired_object.c_str());
				m_act_perceptionServer->setAborted(
					ApcPerceptionResult());
				m_sub_pointCloud.shutdown();
				return;
			}
			objectInfo = &it->second;
		}

		NODELET_INFO("shelf perception");

		// Shadow filter
		typedef pcl::PointXYZRGB Point;
		Point min,max;
		PointCloudXYZRGB::Ptr shadowFilteredCloud(new PointCloudXYZRGB);
		{
			NODELET_INFO("shadow filter...");
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

			pcl::IntegralImageNormalEstimation<Point, pcl::Normal> ne;
			ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
			ne.setMaxDepthChangeFactor(0.1);  // m_param_ne_maxDepthChange()
			ne.setNormalSmoothingSize(10.0);  // m_param_ne_smoothing()
			ne.setInputCloud(cloud);
			ne.compute(*normals);

			pcl::ShadowPoints<Point, pcl::Normal> shadow;
			shadow.setNormals(normals);
			shadow.setInputCloud(cloud);
			shadow.setKeepOrganized(true);
			shadow.setThreshold(0.1);  // m_param_shadow_threshold()
			shadow.filter(*shadowFilteredCloud);

			shadowFilteredCloud->header = cloud->header;
			shadowFilteredCloud->width = cloud->width;
			shadowFilteredCloud->height = cloud->height;
		}

		// Do registration
		// registration needs the pointcloundXYZ and msg is pointcloundXYZRGB
		// FIXME: make ApcShelfFeatures::doRegistration template
		PointCloudXYZ::Ptr msgXYZ (new PointCloudXYZ);
		pcl::copyPointCloud(*shadowFilteredCloud, *msgXYZ);

		Eigen::Affine3f shelfFrame;
		tf::Transform tfShelfFrame;

		// SEGMENTATION
		cv::Mat_<uint8_t> shelfMask(cloud->height, cloud->width, 255);

		m_shelfBackgroundSegmentation.setBinMask(shelfMask);
		auto result = m_shelfBackgroundSegmentation.segmentShelf(cloud);
/*
		// Erode the shelf mask (we want to discard points rather than use them for ICP)
		{
			constexpr int erosion_size = 10;
			cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
				cv::Size(2*erosion_size + 1, 2*erosion_size+1),
				cv::Point(erosion_size, erosion_size)
			);
			cv::erode(result.mask, result.mask, element);
		}*/

		cv::imwrite("/tmp/shelf_segmentation.png", result.mask);

		// REGISTRATION
		m_tfCamera2Shelf = m_shelfFeatures.doRegistration(msgXYZ, m_kinematicCamera2World, result.mask, &shelfFrame);

		m_pub_filteredCloud.publish(m_shelfFeatures.filteredCloud());
		m_pub_filteredShelfCloud.publish(m_shelfFeatures.filteredMeshCloud());
		m_pub_transformedCloud.publish(m_shelfFeatures.transformedCloud());

		NODELET_INFO_STREAM("Got shelf frame:\n" << shelfFrame.matrix());

		tf::transformEigenToTF(shelfFrame.cast<double>(), tfShelfFrame);

		m_tf_broadcaster.sendTransform(tf::StampedTransform(
				tfShelfFrame, pcl_conversions::fromPCL(cloud->header.stamp),
				"world", "perception_shelf"));

		m_cloudInContainer.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::transformPointCloud(*shadowFilteredCloud, *m_cloudInContainer, m_tfCamera2Shelf);

		{
			pcl::PointXYZRGB min,max;

			pcl::getMinMax3D(*shadowFilteredCloud,min,max);
			NODELET_INFO_STREAM("msg Min Max" << min << " " << max);

			pcl::getMinMax3D(*m_cloudInContainer,min,max);
			NODELET_DEBUG_STREAM("Shelf Frame Min Max " << min << " " << max);

			apc_capture::BoxCoordinates m_boxmsg = m_shelfFeatures.getBoxCoordinates();
			NODELET_DEBUG_STREAM(" Box Coordinates X "<<m_boxmsg.boxCoordinates[0]<<" "<<m_boxmsg.boxCoordinates[1]);
			NODELET_DEBUG_STREAM(" Box Coordinates Y "<<m_boxmsg.boxCoordinates[2]<<" "<<m_boxmsg.boxCoordinates[3]);
			NODELET_DEBUG_STREAM(" Box Coordinates Z "<<m_boxmsg.boxCoordinates[4]<<" "<<m_boxmsg.boxCoordinates[5]);
		}

		// transform pointclound to shelf frame

		apc_capture::BoxCoordinates m_boxmsg = m_shelfFeatures.getBoxCoordinates();
		{
			// filter the points that lie out side of the box in focus
			float error_limit = 0.05f;

			pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[0] - error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[1] + error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[2] - error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[3] + error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[4] - error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[5] + error_limit)));

			pcl::ConditionalRemoval<pcl::PointXYZRGB> removal (range_cond);
			removal.setInputCloud (m_cloudInContainer);
			removal.setKeepOrganized(true);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudT (new pcl::PointCloud<pcl::PointXYZRGB>);
			removal.filter (*cloudT);
			m_cloudInContainer = cloudT;
		}

		m_shelfFeatures.setPointCloud(m_cloudInContainer);

		// create bin Mask
		Eigen::Affine3f shelf2CameraFrame = m_tfCamera2Shelf.inverse();
		computeBinMask(shelf2CameraFrame);

		// If the bin mask is zero, cancel.
		{
			bool allZero = true;
			auto& binMask = m_featureImages["binMask"];

			for(int y = 0; y < binMask.rows; ++y)
			{
				for(int x = 0; x < binMask.cols; ++x)
				{
					if(binMask.at<uint8_t>(y,x))
					{
						allZero = false;
						break;
					}
				}
			}

			if(allZero)
			{
				ROS_ERROR("Bin mask is zero, waiting for next frame");
				return;
			}
		}

		// compute all features for shelf
		extractRGBimage(shadowFilteredCloud);

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

		cv::Mat_<cv::Vec3b> hha = m_shelfFeatures.computeHHA(m_tfCamera2Shelf, cloud);

		switch(m_recognitionModule)
		{
			case RECOG_DENSECAP:
			{
				m_shelf_denseCap.setBinMask(m_featureImages["binMask"]);
				auto result = m_shelf_denseCap.compute(cloud, normalCloud, m_tfCamera2Shelf);

				NODELET_INFO("DenseCap response: %f", result.response);

				cv::Rect boundingBox;
				Eigen::Vector2f topLeft = result.center - result.size/2;
				boundingBox.x = topLeft.x();
				boundingBox.y = topLeft.y();
				boundingBox.width  = result.size[0];
				boundingBox.height = result.size[1];

				capBoundingBox(boundingBox, cloud->width, cloud->height);
				NODELET_INFO_STREAM("bounding box: " << boundingBox);

				{
					cv::Mat_<cv::Vec3b> rgb(cloud->height, cloud->width);

					for(unsigned int y = 0; y < cloud->height; ++y)
					{
						for(unsigned int x = 0; x <cloud->width; ++x)
						{
							auto& p = (*cloud)(cloud->width - 1 - x, cloud->height - 1 - y);
							rgb(y,x) = cv::Vec3b(
								p.b, p.g, p.r
							);
						}
					}

					cv::Point upperLeft(
						1920 - 1 - boundingBox.x,
						1080 - 1 - boundingBox.y
					);
					cv::Point lowerRight(
						1920 - 1 - boundingBox.x - boundingBox.width,
						1080 - 1 - boundingBox.y - boundingBox.height
					);

					cv::rectangle(rgb, upperLeft, lowerRight, cv::Scalar(0, 0, 255), 3);

					sensor_msgs::ImagePtr img =
						cv_bridge::CvImage(
							pcl_conversions::fromPCL(cloud->header),
							"bgr8", rgb
						).toImageMsg();

					m_pub_vis.publish(img);
				}

				if(result.response > -0.6)
				{
					// Create object mask from bounding box
					cv::Mat_<uint8_t> objectMask(cloud->height, cloud->width, (uint8_t)0);
					{
						cv::Mat_<uint8_t> mask(boundingBox.height, boundingBox.width, (uint8_t)255);
						mask.copyTo(objectMask(boundingBox));
					}

					m_DenseCapResultMask = objectMask;
					auto poses = findShelfGraspPose(
						cloud, normalCloud, m_tfCamera2Shelf, boundingBox,
						objectMask, *objectInfo);
					if(!poses.empty())
					{
						perceptionResult.grasp_poses = poses;
						perceptionResult.box_coordinates = m_shelfFeatures.getBoxCoordinates();
						perceptionResult.grasp_object = m_act_goal->desired_object;
						m_act_perceptionServer->setSucceeded(perceptionResult);
						m_sub_pointCloud.shutdown();
					}
				}

				break;
			}
			case RECOG_SEGMENTATION:
			case RECOG_SEGMENTATION_DENSECAP:
			{
				m_shelfSegmentation.setBinMask(m_featureImages["binMask"]);

				std::set<std::string> uniqueObjects;
				for(auto& obj : m_act_goal->candidate_objects)
					uniqueObjects.insert(obj);

				std::vector<std::string> segmentationObjects(uniqueObjects.begin(), uniqueObjects.end());

				auto it = std::find(
					segmentationObjects.begin(), segmentationObjects.end(), m_act_goal->desired_object
				);
				if(it == segmentationObjects.end())
				{
					ROS_ERROR("Invalid goal object");
					m_act_perceptionServer->setAborted(ApcPerceptionResult());
					m_sub_pointCloud.shutdown();
					return;
				}

				bool useDenseCap = (m_recognitionModule == RECOG_SEGMENTATION_DENSECAP) && !m_act_goal->unsure_candidates;
				std::future<apc_densecap::APCDenseCap::MultiResult> densecapFuture;
				if(useDenseCap)
				{
					m_shelf_denseCap.setBinMask(m_featureImages["binMask"]);

					densecapFuture = std::async(std::launch::async, std::bind(
						&apc_densecap::APCDenseCap::computeMulti, &m_shelf_denseCap,
						cloud, normalCloud, Eigen::Affine3f::Identity()
					));
				}

				int idx = it - segmentationObjects.begin() + 1; // 0 is shelf

				NODELET_INFO("==================================================");
				NODELET_INFO("Segmentation for %s...", m_act_goal->desired_object.c_str());
				auto result = m_shelfSegmentation.segmentObjects(cloud, hha, segmentationObjects);

				cv::imwrite("/tmp/segmentation_out.png", result.mask);

				cv::Mat_<uint8_t> visConf;
				result.confidence.convertTo(visConf, CV_8UC1, 255);

				cv::imwrite("/tmp/segmentation_rgb.png", m_featureImages["rgb"]);
				cv::imwrite("/tmp/segmentation_confidence.png", visConf);
				cv::imwrite("/tmp/segmentation_hha.png", hha);

				if(useDenseCap)
				{
					// Refine segmentation mask using densecap
					auto densecapResult = densecapFuture.get();

					cv::Mat_<float> boxChannel(cloud->height, cloud->width, 1.0f);

					const float alpha = 1.0;

					for(auto& pair : densecapResult)
					{
						double min, max;
						cv::minMaxLoc(pair.second.likelihood, &min, &max);

						auto it = std::find(segmentationObjects.begin(), segmentationObjects.end(), pair.first);
						int idx = it - segmentationObjects.begin();

						for(size_t y = 0; y < cloud->height; ++y)
						{
							for(size_t x = 0; x < cloud->width; ++x)
							{
								float p = (1.0f - alpha) + alpha * pair.second.likelihood(y,x) / max;

								result.objectPrediction(idx + 1, y, x) *= p;
								boxChannel(y,x) *= (1.0f - alpha) + alpha * (1.0 - pair.second.likelihood(y,x) / max);
							}
						}
					}

					// Re-normalize and make segmentation decision
					for(size_t y = 0; y < cloud->height; ++y)
					{
						for(size_t x = 0; x < cloud->width; ++x)
						{
							float sumconf = boxChannel(y,x) * result.objectPrediction(0,y,x);
							float maxconf = boxChannel(y,x) * result.objectPrediction(0,y,x);
							int maxidx = 0;

							for(size_t c = 0; c < segmentationObjects.size(); ++c)
							{
								float conf = result.objectPrediction(c+1, y,x);
								sumconf += conf;

								if(conf > maxconf)
								{
									maxconf = conf;
									maxidx = c+1;
								}
							}

							for(size_t c = 0; c < segmentationObjects.size()+1; ++c)
								result.objectPrediction(c,y,x) /= sumconf;

							result.mask(y,x) = maxidx;
						}
					}

					cv::imwrite((m_currentSavePath / "mask_combined.png").string(), result.mask);

					timings.addCheckpoint("densecap");
				}

				cv::bitwise_and(result.mask, m_featureImages["binMask"], result.mask);

				cv::Mat_<uint8_t> binaryMask;
				binaryMask = 255 * (result.mask == idx);

				cv::imwrite("/tmp/segmentation_binary.png", binaryMask);

				NODELET_INFO("Connected components...");
				cv::Mat_<int32_t> labels;
				cv::Mat_<int32_t> stats;
				cv::Mat_<double> centroids;

				int num_labels = ::connectedComponentsWithStats(binaryMask, labels, stats, centroids, 4, CV_32S);

				for(int i = 0; i < num_labels; ++i)
				{
					NODELET_INFO("comp: %d", stats(i, CC_STAT_AREA));
				}

				cv::imwrite("/tmp/segmentation_labels.png", labels);

				if(num_labels <= 1)
				{
					NODELET_WARN("No segments :-(");
					perceptionResult.grasp_poses.clear();
					perceptionResult.box_coordinates = m_shelfFeatures.getBoxCoordinates();
					m_act_perceptionServer->setSucceeded(perceptionResult);
					m_sub_pointCloud.shutdown();
					return;
				}

				// Find biggest component
				int maxArea = 0;
				int maxIdx = -1;

				for(int i = 1; i < num_labels; ++i)
				{
					int area = stats(i, CC_STAT_AREA);
					if(area > maxArea)
					{
						maxArea = area;
						maxIdx = i;
					}
				}

				// Extract segment mask
				cv::Mat_<uint8_t> segment;
				{
					segment = 255 * (labels == maxIdx);
				}
				cv::imwrite("/tmp/segmentation_segment.png", segment);

				// Erode the segment mask
				if(objectInfo->erode != 0)
				{
					const int erosion_size = objectInfo->erode;
					cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
						cv::Size(2*erosion_size + 1, 2*erosion_size+1),
						cv::Point(erosion_size, erosion_size)
					);
					cv::erode(segment, segment, element);
				}

				auto collisionMask = computeCollisionMask(
					result.mask, segment, idx);
				cv::imwrite("/tmp/segmentation_collision_mask.png", collisionMask);

				cv::Mat_<float> collisionDistances;
				cv::distanceTransform(collisionMask, collisionDistances,
					CV_DIST_L2, CV_DIST_MASK_PRECISE);
				{
					cv::Mat_<uint8_t> vis;
					collisionDistances.convertTo(vis, CV_8UC1, 255.0f/collisionDistances.cols);
					cv::imwrite("/tmp/segmentation_collision_distances.png", vis);
				}

				{
					constexpr int extractRow = (int)(0 / collisionMaskMeterPerPixel);
					// 0.125m/0.002m : y-offset / mask-resolution
					NODELET_DEBUG("Set extract row to %d", extractRow);
					float maxDist = 0.0f;
					int maxY = 0;

					for(auto col = 0; col < collisionDistances.cols; ++col)
					{
						const auto dist = collisionDistances(extractRow, col);
// 						if(dist == maxDist)
// 						{
// 							auto center = collisionDistances.cols / 2;
// 							if(abs(col - center) < abs(maxY - center))
// 								maxY = col;
// 						}
						if(dist > maxDist)
						{
							maxDist = dist;
							maxY = col;
						}
					}
					// Transform column to world frame point
					constexpr uint8_t MY = 3;
					float shelfMaxY =
						m_shelfFeatures.getBoxCoordinates().boxCoordinates[MY];
					float posY = shelfMaxY - (maxY * collisionMaskMeterPerPixel);
					perceptionResult.retractPositionY = posY;
					NODELET_INFO("Got extract Y Offset : %d/%d [col], %f [box], %f [world]",
						maxY, collisionDistances.cols, maxY * collisionMaskMeterPerPixel, posY);
				}

				// Build bounding box
				cv::Rect boundingBox(
					stats(maxIdx, CC_STAT_LEFT),
					stats(maxIdx, CC_STAT_TOP),
					stats(maxIdx, CC_STAT_WIDTH),
					stats(maxIdx, CC_STAT_HEIGHT)
				);

				float confidence = 0;
				unsigned int numPix = 0;
				for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
				{
					for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
					{
						if(!segment(y,x))
							continue;

						confidence += result.confidence(y,x);
						numPix++;
					}
				}
				confidence /= numPix;

				ROS_INFO("apc_segmentation confidence: %f", confidence);

				{
					cv::Mat_<cv::Vec3b> rgb(cloud->height, cloud->width);

					for(unsigned int y = 0; y < cloud->height; ++y)
					{
						for(unsigned int x = 0; x <cloud->width; ++x)
						{
							auto& p = (*cloud)(cloud->width - 1 - x, cloud->height - 1 - y);
							if(segment(cloud->height - y - 1, cloud->width - 1 - x))
							{
								rgb(y,x) = cv::Vec3b(
									p.b/2, p.g, p.r/2
								);
							}
							else
							{
								rgb(y,x) = cv::Vec3b(
									p.b, p.g, p.r
								);
							}
						}
					}

					cv::Point upperLeft(
						1920 - 1 - boundingBox.x,
						1080 - 1 - boundingBox.y
					);
					cv::Point lowerRight(
						1920 - 1 - boundingBox.x - boundingBox.width,
						1080 - 1 - boundingBox.y - boundingBox.height
					);

					cv::rectangle(rgb, upperLeft, lowerRight, cv::Scalar(0, 0, 255), 3);

					sensor_msgs::ImagePtr img =
						cv_bridge::CvImage(
							pcl_conversions::fromPCL(cloud->header),
							"bgr8", rgb
						).toImageMsg();

					m_pub_vis.publish(img);
				}

				// Publish segment cloud
				{
					auto segmentCloud = boost::make_shared<PointCloudXYZRGB>();

					segmentCloud->header = cloud->header;
					segmentCloud->reserve(stats(maxIdx, CC_STAT_AREA));

					for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
					{
						for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
						{
							if(!segment(y,x))
								continue;

							segmentCloud->push_back((*cloud)(x,y));
						}
					}

					m_pub_segmentCloud.publish(segmentCloud);
				}

				m_CNNResultMask = segment;

				timings.addCheckpoint("before pose estimation");

				std::vector<geometry_msgs::PoseStamped> poses;
				bool objectStanding = false;

				if(m_registration && objectInfo->registration)
				{
					poses = findShelfRegistrationGraspPose(
						cloud, boundingBox, segment,
						m_tfCamera2Shelf, *objectInfo
					);

					timings.addCheckpoint("registration");
				}

				if(poses.empty())
				{
					poses = findShelfGraspPose(
						cloud, normalCloud, m_tfCamera2Shelf,
						boundingBox, segment, *objectInfo, &objectStanding);

					timings.addCheckpoint("grasp");
				}

				if(!poses.empty())
				{
					perceptionResult.grasp_poses = poses;
					perceptionResult.box_coordinates = m_shelfFeatures.getBoxCoordinates();
					perceptionResult.grasp_object = m_act_goal->desired_object;
					perceptionResult.object_standing = objectStanding;
					m_act_perceptionServer->setSucceeded(perceptionResult);
					m_sub_pointCloud.shutdown();
				}

				break;
			}
		}


		/*
		 * Draft code: ranking 3 models for perception (CNN, DenseCap, RBO)
		 * Assumptions: we have the results from the 3 models (how to run this in parallel?)
		 * Idea: case 1: DenseCap bounding box is fine:  compare the percentage of overlap between CNN-Densecap and CNN-RBO and go with the model that has maximum overlap
		 * 		 case 2: DenseCap bounding box is bad:   Compute the model overlaps but if the both the models (RBO and CNN) has very little overlap (less than "5%"), then discard DenseCap and
		 * 		         go with the overlap between CNN and RBO
		 * 	Next steps : How to use the confidence score from DenseCap and CNN while ranking?
 		 */
/*		if (m_RBOResultMask.empty() || m_DenseCapResultMask.empty() || m_CNNResultMask.empty())
		{
			NODELET_INFO("Segmentation mask for all three models is not available");
		}
		else
		{
			cv::Mat and_DenseCap_RBO,and_Densecap_CNN;
			int sum_And_DenseCap_RBO, sum_And_Densecap_CNN, sum_DenseCap_Mask;

			cv::bitwise_and(m_DenseCapResultMask,m_RBOResultMask,and_DenseCap_RBO);
			cv::bitwise_and(m_DenseCapResultMask,m_CNNResultMask,and_Densecap_CNN);

			sum_And_DenseCap_RBO = cv::sum(and_DenseCap_RBO)[0];
			sum_And_Densecap_CNN = cv::sum(and_Densecap_CNN)[0];
			sum_DenseCap_Mask    = cv::sum(m_DenseCapResultMask)[0];
			if (sum_And_DenseCap_RBO > 0.05 * sum_DenseCap_Mask || sum_And_Densecap_CNN > 0.05 * sum_DenseCap_Mask)
			{
				if (sum_And_DenseCap_RBO > sum_And_Densecap_CNN)
					m_rankedResultMask = and_DenseCap_RBO;
				else
					m_rankedResultMask = and_Densecap_CNN;
			}
			else
			{
				NODELET_INFO("DenseCap has very little overlap with other models");
				cv::bitwise_and(m_CNNResultMask,m_RBOResultMask,m_rankedResultMask);
				if (cv::sum(m_rankedResultMask)[0] < 0.1 * cv::sum(m_CNNResultMask)[0])
				{
					NODELET_INFO(" Very little overlap b/w CNN and RBO");
					m_rankedResultMask = m_CNNResultMask;
				}
			}

		}*/

	}
	else // tote
	{

		NODELET_INFO("tote perception");

		// Shadow filter
		typedef pcl::PointXYZRGB Point;
		PointCloudXYZRGB::Ptr shadowFilteredCloud(new PointCloudXYZRGB);
		{
			NODELET_INFO("shadow filter...");
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

			pcl::IntegralImageNormalEstimation<Point, pcl::Normal> ne;
			ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
			ne.setMaxDepthChangeFactor(0.1);  // m_param_ne_maxDepthChange()
			ne.setNormalSmoothingSize(10.0);  // m_param_ne_smoothing()
			ne.setInputCloud(cloud);
			ne.compute(*normals);

			pcl::ShadowPoints<Point, pcl::Normal> shadow;
			shadow.setNormals(normals);
			shadow.setInputCloud(cloud);
			shadow.setKeepOrganized(true);
			shadow.setThreshold(0.1);  // m_param_shadow_threshold()
			shadow.filter(*shadowFilteredCloud);

			shadowFilteredCloud->header = cloud->header;
			shadowFilteredCloud->width = cloud->width;
			shadowFilteredCloud->height = cloud->height;
		}

		timings.addCheckpoint("shadow");



		PointCloudXYZ::Ptr msgXYZ (new PointCloudXYZ);
		pcl::copyPointCloud(*shadowFilteredCloud, *msgXYZ);

		if(!m_tf->waitForTransform("tote", cloud->header.frame_id,  pcl_conversions::fromPCL(cloud->header.stamp), ros::Duration(5.0)))
		{
			NODELET_ERROR(" The transform wait from cloud to tote timeout ");
			return;
		}
		try
		{
			m_tf->lookupTransform(
				"tote", cloud->header.frame_id,
				pcl_conversions::fromPCL(cloud->header.stamp), m_tfTransCam2Tote
			);
		}
		catch(tf::TransformException& e)
		{
			NODELET_ERROR("Could not get input transform: %s", e.what());
			return;
		}

		Eigen::Affine3d toteFrame;
		tf::transformTFToEigen(m_tfTransCam2Tote, toteFrame);
		m_cloudInContainer = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::transformPointCloud(*shadowFilteredCloud,*m_cloudInContainer,toteFrame);
		pcl::PointXYZRGB min,max;


		{
			// filter the points that lie out side of the box in focus
			float error_limit = 0.02f;

			pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, -0.30 - error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT,  0.30 + error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, -0.23 - error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT,  0.23 + error_limit)));
			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT,  0    - error_limit)));

			// This is dangerous, there are many objects sticking out of the tote!
// 			range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT,  0.25 + error_limit)));

			pcl::ConditionalRemoval<pcl::PointXYZRGB> removal (range_cond);
			removal.setInputCloud (m_cloudInContainer);
			removal.setKeepOrganized(true);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudT (new pcl::PointCloud<pcl::PointXYZRGB>);
			removal.filter (*cloudT);
			m_cloudInContainer = cloudT;
		}

		pcl::getMinMax3D(*m_cloudInContainer,min,max);
		std::cout<<"tote Frame Min Max"<<min<<" "<<max<<std::endl;

		timings.addCheckpoint("cloudInContainer");

		// Compute normals
		pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
		{
			pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

			ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
			ne.setMaxDepthChangeFactor(1.0f);
			ne.setNormalSmoothingSize(160.0f);

			ne.setInputCloud(cloud);
			ne.compute(*normalCloud);
		}
		pcl::PointCloud<pcl::Normal>::Ptr normalHHACloud(new pcl::PointCloud<pcl::Normal>);
		{
			pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

			ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
			ne.setMaxDepthChangeFactor(0.8f);
			ne.setNormalSmoothingSize(30.0f);

			ne.setInputCloud(cloud);
			ne.compute(*normalHHACloud);
		}

		timings.addCheckpoint("normals");

		// create tote Mask
		Eigen::Affine3f toteFrameFloat   = toteFrame.cast<float>();

		m_toteFeatures.setPointCloud(m_cloudInContainer);
		computeToteMask(toteFrameFloat.inverse());

		timings.addCheckpoint("tote mask");

		// compute features for tote
		extractRGBimage(shadowFilteredCloud);

		m_featureImages["hha"] = m_toteFeatures.computeHHA(toteFrameFloat, cloud, normalHHACloud);

		timings.addCheckpoint("HHA + RGB");

		switch(m_recognitionModule)
		{
			case RECOG_DENSECAP:
			{
				cv::imwrite("/tmp/tote_mask.png", m_featureImages["binMask"]);

				m_tote_denseCap.setBinMask(m_featureImages["binMask"]);
				auto results = m_tote_denseCap.computeMulti(cloud, normalCloud, toteFrameFloat);

				NODELET_INFO("DenseCap response:");
				for(auto& obj : results)
				{
					NODELET_INFO(" - %30s: %f", obj.first.c_str(), obj.second.response);
				}

				{
					cv::Mat_<cv::Vec3b> rgb(cloud->height, cloud->width);

					for(unsigned int y = 0; y < cloud->height; ++y)
					{
						for(unsigned int x = 0; x <cloud->width; ++x)
						{
							auto& p = (*cloud)(cloud->width - 1 - x, cloud->height - 1 - y);
							rgb(y,x) = cv::Vec3b(
								p.b, p.g, p.r
							);
						}
					}

					unsigned int colorIdx = 0;

					for(auto& obj : results)
					{
						auto& result = obj.second;

						if(result.response < -2)
							continue;

						auto& color = COLORS[colorIdx % COLORS.size()];

						cv::Rect boundingBox;
						Eigen::Vector2f topLeft = result.center - result.size/2;
						boundingBox.x = topLeft.x();
						boundingBox.y = topLeft.y();
						boundingBox.width  = result.size[0];
						boundingBox.height = result.size[1];

						capBoundingBox(boundingBox, cloud->width, cloud->height);
						NODELET_INFO_STREAM("bounding box: " << boundingBox);

						cv::Point upperLeft(
							1920 - 1 - boundingBox.x,
							1080 - 1 - boundingBox.y
						);
						cv::Point lowerRight(
							1920 - 1 - boundingBox.x - boundingBox.width,
							1080 - 1 - boundingBox.y - boundingBox.height
						);

						cv::rectangle(rgb, upperLeft, lowerRight, color, 3);
						{
							std::stringstream ss;
							ss << obj.first << " " << obj.second.response;

							cv::putText(
								rgb, ss.str(), cv::Point(lowerRight.x, upperLeft.y),
								cv::FONT_HERSHEY_SIMPLEX, 1, color
							);
						}

						colorIdx++;
					}

					sensor_msgs::ImagePtr img =
						cv_bridge::CvImage(
							pcl_conversions::fromPCL(cloud->header),
							"bgr8", rgb
						).toImageMsg();

					m_pub_vis.publish(img);
				}

				// Find maximum response
				apc_densecap::APCDenseCap::Result maxResult{};
				std::string maxName;
				maxResult.response = -std::numeric_limits<float>::infinity();
				for(auto& obj : results)
				{
					if(obj.second.response > maxResult.response)
					{
						maxResult = obj.second;
						maxName = obj.first;
					}
				}

				const apc_objects::APCObject* maxInfo;
				{
					auto it = apc_objects::objects().find(maxName);
					if(it == apc_objects::objects().end())
					{
						NODELET_ERROR("Maximum respones object '%s' not found.", maxName.c_str());
						m_act_perceptionServer->setAborted(
							ApcPerceptionResult());
						return;
					}
					maxInfo = &it->second;
				}

				if(maxResult.response > -2)
				{
					cv::Rect boundingBox;
					Eigen::Vector2f topLeft = maxResult.center - maxResult.size/2;
					boundingBox.x = topLeft.x();
					boundingBox.y = topLeft.y();
					boundingBox.width  = maxResult.size[0];
					boundingBox.height = maxResult.size[1];

					capBoundingBox(boundingBox, cloud->width, cloud->height);
					NODELET_INFO_STREAM("bounding box: " << boundingBox);

					// Create object mask from bounding box
					cv::Mat_<uint8_t> objectMask(cloud->height, cloud->width, (uint8_t)0);
					{
						cv::Mat_<uint8_t> mask(boundingBox.height, boundingBox.width, (uint8_t)255);
						mask.copyTo(objectMask(boundingBox));
					}

					auto poses = findToteGraspPose(
						cloud, normalCloud, toteFrameFloat,
						boundingBox, objectMask, *maxInfo);
					if(!poses.empty())
					{
						perceptionResult.grasp_poses = poses;
						perceptionResult.grasp_object = maxName;
						perceptionResult.box_coordinates = m_shelfFeatures.getBoxCoordinates();
						m_act_perceptionServer->setSucceeded(perceptionResult);
						m_sub_pointCloud.shutdown();
					}
				}

				break;
			}
			case RECOG_SEGMENTATION:
			case RECOG_SEGMENTATION_DENSECAP:
			{
				m_toteSegmentation.setBinMask(m_featureImages["binMask"]);

				cv::imwrite("/tmp/tote_rgb.png", m_featureImages["rgb"]);
				cv::imwrite("/tmp/tote_mask.png", m_featureImages["binMask"]);
				cv::imwrite("/tmp/tote_hha.png", m_featureImages["hha"]);

				std::set<std::string> uniqueObjects;
				for(auto& obj : m_act_goal->candidate_objects)
					uniqueObjects.insert(obj);

				std::vector<std::string> segmentationObjects(uniqueObjects.begin(), uniqueObjects.end());

				bool useDenseCap = (m_recognitionModule == RECOG_SEGMENTATION_DENSECAP) && !m_act_goal->unsure_candidates;
				std::future<apc_densecap::APCDenseCap::MultiResult> densecapFuture;
				if(useDenseCap)
				{
					m_tote_denseCap.setBinMask(m_featureImages["binMask"]);

					densecapFuture = std::async(std::launch::async, std::bind(
						&apc_densecap::APCDenseCap::computeMulti, &m_tote_denseCap,
						cloud, normalCloud, Eigen::Affine3f::Identity()
					));
				}

				NODELET_INFO("Segmentation...");
				auto result = m_toteSegmentation.segmentObjects(cloud, m_featureImages["hha"], segmentationObjects);

				timings.addCheckpoint("segmentation");

				cv::imwrite((m_currentSavePath / "mask_segmentation.png").string(), result.mask);

// 				cv::Mat_<uint8_t> visConf;
// 				result.confidence.convertTo(visConf, CV_8UC1, 255);
// 				cv::imwrite("/tmp/segmentation_confidence.png", visConf);

				if(useDenseCap)
				{
					// Refine segmentation mask using densecap
					auto densecapResult = densecapFuture.get();

					cv::Mat_<float> boxChannel(cloud->height, cloud->width, 1.0f);

					const float alpha = 1.0;

					for(auto& pair : densecapResult)
					{
						double min, max;
						cv::minMaxLoc(pair.second.likelihood, &min, &max);

						auto it = std::find(segmentationObjects.begin(), segmentationObjects.end(), pair.first);
						int idx = it - segmentationObjects.begin();

						for(size_t y = 0; y < cloud->height; ++y)
						{
							for(size_t x = 0; x < cloud->width; ++x)
							{
								float p = (1.0f - alpha) + alpha * pair.second.likelihood(y,x) / max;

								result.objectPrediction(idx + 1, y, x) *= p;
								boxChannel(y,x) *= (1.0f - alpha) + alpha * (1.0 - pair.second.likelihood(y,x) / max);
							}
						}
					}

					// Re-normalize and make segmentation decision
					for(size_t y = 0; y < cloud->height; ++y)
					{
						for(size_t x = 0; x < cloud->width; ++x)
						{
							float sumconf = boxChannel(y,x) * result.objectPrediction(0,y,x);
							float maxconf = boxChannel(y,x) * result.objectPrediction(0,y,x);
							int maxidx = 0;

							for(size_t c = 0; c < segmentationObjects.size(); ++c)
							{
								float conf = result.objectPrediction(c+1, y,x);
								sumconf += conf;

								if(conf > maxconf)
								{
									maxconf = conf;
									maxidx = c+1;
								}
							}

							for(size_t c = 0; c < segmentationObjects.size()+1; ++c)
								result.objectPrediction(c,y,x) /= sumconf;

							result.mask(y,x) = maxidx;
						}
					}

					cv::imwrite((m_currentSavePath / "mask_combined.png").string(), result.mask);

					timings.addCheckpoint("densecap");
				}

				cv::bitwise_and(result.mask, m_featureImages["binMask"], result.mask);

				std::vector<float> segmentationObjectLikelihoods(segmentationObjects.size(), 0.0f);

				timings.addCheckpoint("vis");

				// Compute connected components for each class
				struct Candidate
				{
					float height = -0.05f;
					float area = 0.0f;
					cv::Mat_<int32_t> stats;
					int object = -1;
					int label;
					cv::Mat_<int32_t> mask;
					float likelihood = 0.0f;
					int collectiveScore = 0;
				};

				std::vector<Candidate> cand_closeToWall;
				std::vector<Candidate> cand_awayFromWall;

				for(size_t i = 0; i < segmentationObjects.size(); ++i)
				{
					auto it = apc_objects::objects().find(segmentationObjects[i]);
					if(it == apc_objects::objects().end())
						throw std::logic_error("Unknown object");

					if(it->second.stow_forbidden)
						continue;

					cv::Mat_<uint8_t> binaryMask;
					binaryMask = 255 * (result.mask == i+1);

					NODELET_INFO("Connected components for %s...", segmentationObjects[i].c_str());
					cv::Mat_<int32_t> labels;
					cv::Mat_<int32_t> stats;
					cv::Mat_<double> centroids;

					int num_labels = ::connectedComponentsWithStats(binaryMask, labels, stats, centroids, 4, CV_32S);

					Candidate biggestCloseToWall;
					Candidate biggestAwayFromWall;

					for(int j = 1; j < num_labels; ++j)
					{
						if(stats(j, CC_STAT_AREA) < m_param_toteMinPoints())
							continue;

						// Calculate robust height measure and distance to wall
						std::vector<float> heights;
						float dist2Wall = 0;
						float likelihood = 0.0f;
						unsigned int numValid = 0;

						for(int y = stats(j, CC_STAT_TOP); y < stats(j, CC_STAT_TOP) + stats(j, CC_STAT_HEIGHT); ++y)
						{
							for(int x = stats(j, CC_STAT_LEFT); x < stats(j, CC_STAT_LEFT) + stats(j, CC_STAT_WIDTH); ++x)
							{
								if(labels(y,x) == j)
								{
									auto& p = (*m_cloudInContainer)(x,y);
									if(!std::isfinite(p.z))
										continue;

									float dist = std::min({
											std::abs(std::abs(p.x) - tote_X),
											std::abs(std::abs(p.y) - tote_Y)
											});

									dist2Wall += dist;
									likelihood += result.objectPrediction(i+1, y, x);
									numValid++;

									heights.push_back(p.z);
								}
							}
						}

						if(heights.empty())
							continue;

						std::sort(heights.begin(), heights.end());
						float height = heights[0.9*heights.size()];

						dist2Wall /= numValid;
						likelihood /= numValid;

						NODELET_INFO("Object %s (area %d) height: %f, dist2Wall: %f", segmentationObjects[i].c_str(), stats(j, CC_STAT_AREA), height, dist2Wall);

						if(dist2Wall > 0.04)
						{
							if(stats(j, CC_STAT_AREA) > biggestAwayFromWall.area)
							{
								NODELET_INFO("... which is the new 'away from wall' maximum.");
								biggestAwayFromWall.object = i;
								biggestAwayFromWall.label = j;
								biggestAwayFromWall.stats = stats.row(j);
								biggestAwayFromWall.mask = labels;
								biggestAwayFromWall.height = height;
								biggestAwayFromWall.area = stats(j, CC_STAT_AREA);
								biggestAwayFromWall.likelihood = likelihood;
							}
						}
						else
						{
							if(stats(j, CC_STAT_AREA) > biggestCloseToWall.area)
							{
								NODELET_INFO("... which is the new 'close to wall' maximum.");
								biggestCloseToWall.object = i;
								biggestCloseToWall.label = j;
								biggestCloseToWall.stats = stats.row(j);
								biggestCloseToWall.mask = labels;
								biggestCloseToWall.height = height;
								biggestCloseToWall.area = stats(j, CC_STAT_AREA);
								biggestCloseToWall.likelihood = likelihood;
							}
						}
					}

					float likelihood = 0.0f;
					if(biggestAwayFromWall.object != -1)
						likelihood = biggestAwayFromWall.likelihood;
					else if(biggestCloseToWall.object != -1)
						likelihood = biggestAwayFromWall.likelihood;

					segmentationObjectLikelihoods[i] = likelihood;

					if(biggestAwayFromWall.object != -1)
						cand_awayFromWall.push_back(biggestAwayFromWall);
					if(biggestCloseToWall.object != -1)
						cand_closeToWall.push_back(biggestCloseToWall);
				}

				// sort each candidate list separately by area
				auto cmp = [](const Candidate& a, const Candidate& b) {
					return a.area > b.area;
				};


				std::sort(cand_awayFromWall.begin(), cand_awayFromWall.end(), cmp);
				std::sort(cand_closeToWall.begin(), cand_closeToWall.end(), cmp);

				// Concatenate lists
				std::vector<Candidate> allCandidates;
				std::copy(cand_awayFromWall.begin(), cand_awayFromWall.end(), std::back_inserter(allCandidates));
				std::copy(cand_closeToWall.begin(), cand_closeToWall.end(), std::back_inserter(allCandidates));

				for (size_t i = 0; i<allCandidates.size(); ++i )
				{
					allCandidates[i].collectiveScore += i;
				}

				//sort each element by height
				auto heightCmp = [] (const Candidate&a, const Candidate&b){
						return a.height > b.height;
				};

				std::sort(allCandidates.begin(), allCandidates.begin() + allCandidates.size() / 2, heightCmp);

				for (size_t i = 0; i<allCandidates.size(); ++i )
				{
					allCandidates[i].collectiveScore += i;
				}

				// punish the most recent failure object more
				// even more for repetitive failures
				std::vector<int> repetitionCount(segmentationObjects.size(), 0);
				std::vector<int> timeSinceFail(segmentationObjects.size(), 1000);

				//initialize the map
				NODELET_INFO("unwanted objects:");
				for(size_t i = 0; i < m_act_goal->unwanted_objects.size(); ++i)
				{
					auto object = m_act_goal->unwanted_objects[i];

					NODELET_INFO(" - %s", object.c_str());

					int idx = std::find(segmentationObjects.begin(), segmentationObjects.end(), object) - segmentationObjects.begin();
					repetitionCount[idx]++;
					timeSinceFail[idx] = std::min<int>(timeSinceFail[idx], m_act_goal->unwanted_objects.size() - 1 - i);
				}

				auto collectiveScoreCmp = [&] (const Candidate&a, const Candidate&b) {
					int repA = repetitionCount[a.object];
					int repB = repetitionCount[b.object];

					if(repA == repB)
					{
						int timeA = timeSinceFail[a.object];
						int timeB = timeSinceFail[b.object];

						if(timeA == timeB)
						{
							return a.collectiveScore < b.collectiveScore;
						}
						return timeA > timeB;
					}
					return repA < repB;
				};

				std::sort(allCandidates.begin(),allCandidates.end(),collectiveScoreCmp);


				if(allCandidates.empty())
				{
					NODELET_WARN("no object found");
					perceptionResult.grasp_poses.clear();
					perceptionResult.box_coordinates = m_shelfFeatures.getBoxCoordinates();
					m_act_perceptionServer->setSucceeded(perceptionResult);
					m_sub_pointCloud.shutdown();
					return;
				}

				// If the cup is there, try it first, since it has a special motion which only works if the
				// cup is upright.
				{
					auto it = std::find(segmentationObjects.begin(), segmentationObjects.end(), "rolodex_jumbo_pencil_cup");
					int idx = it - segmentationObjects.begin();
					if(it != segmentationObjects.end())
					{
						for(size_t i = 0; i < allCandidates.size(); ++i)
						{
							if(allCandidates[i].object == idx && allCandidates[i].area >= 127*127)
							{
								allCandidates.insert(allCandidates.begin(), allCandidates[i]);
								break;
							}
						}
					}
				}

				NODELET_INFO("Final candidate list:");
				for(auto& candidate : allCandidates)
				{
					NODELET_INFO(" - %30s: %d reps, %d time since last failure, %d collective score",
						segmentationObjects[candidate.object].c_str(),
						repetitionCount[candidate.object],
						timeSinceFail[candidate.object],
						candidate.collectiveScore
					);
				}

				for(auto& candidate : allCandidates)
				{
					std::string maxName = segmentationObjects[candidate.object];
					NODELET_INFO("best: %s with height %f", maxName.c_str(), candidate.height);
					const apc_objects::APCObject* maxInfo;
					{
						auto it = apc_objects::objects().find(maxName);
						if(it == apc_objects::objects().end())
						{
							NODELET_ERROR("Maximum respones object '%s' not found.", maxName.c_str());
							m_act_perceptionServer->setAborted(
									ApcPerceptionResult());
							return;
						}
						maxInfo = &it->second;
					}

					cv::Mat_<uint8_t> segmentMask = 255 * (candidate.mask == candidate.label);

					// Mark the object in the image
					cv::Mat_<cv::Vec3b> vis;
					{
						m_featureImages["rgb"].copyTo(vis);

						auto colors = huslColors(segmentationObjects.size());

						constexpr int VIS_CLASS_HEIGHT = 40;
						char buf[256];

						for(size_t i = 0; i < colors.size(); ++i)
						{
							snprintf(buf, sizeof(buf), "%5.3f %s", segmentationObjectLikelihoods[i], segmentationObjects[i].c_str());

							cv::putText(
								vis, buf, cv::Point(0, (i+1)*VIS_CLASS_HEIGHT),
								cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(colors[i][0], colors[i][1], colors[i][2]), 3
							);
						}

						for(int y = 0; y < vis.rows; ++y)
						{
							for(int x = 0; x < vis.cols; ++x)
							{
								int idx = result.mask(y,x);
								if(idx != 0)
									vis(y,x) = 0.2 * vis(y,x) + 0.8 * colors.at(idx-1);
							}
						}

						// mark boundaries
						cv::Mat_<int16_t> sobel_x;
						cv::Mat_<int16_t> sobel_y;

						cv::Sobel(result.mask, sobel_x, CV_16SC1, 1, 0);
						cv::Sobel(result.mask, sobel_y, CV_16SC1, 0, 1);
						cv::Mat_<int16_t> combined16 = cv::abs(sobel_x + sobel_y);
						cv::Mat_<uint8_t> combined;
						combined16.convertTo(combined, CV_8UC1);
						cv::Mat_<uint8_t> contourMask;
						cv::threshold(combined, contourMask, 0.5, 255.0, cv::THRESH_BINARY);

						vis.setTo(cv::Vec3b(255, 255, 255), contourMask);

						sensor_msgs::ImagePtr img =
							cv_bridge::CvImage(
									pcl_conversions::fromPCL(cloud->header),
									"bgr8", vis
									).toImageMsg();

						m_pub_vis.publish(img);
					}
					{
						cv::Mat eroded;

						{
							constexpr int erosion_size = 1;
							cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
									cv::Size(2*erosion_size + 1, 2*erosion_size+1),
									cv::Point(erosion_size, erosion_size)
									);
							cv::erode(segmentMask, eroded, element);
						}

						cv::Mat outline = segmentMask - eroded;

						vis.setTo(cv::Vec3b(0, 0, 255), outline);
					}

					timings.addCheckpoint("connectedcomp");

					std::vector<geometry_msgs::PoseStamped> poses;
					bool objectStanding = false;

					if(m_registration && maxInfo->registration)
					{
						poses = findRegistrationGraspPose(cloud, segmentMask, toteFrameFloat, *maxInfo);

						timings.addCheckpoint("registration");
					}

					if(poses.empty())
					{
						// Try to compute a grasp
						cv::Rect boundingBox(
								candidate.stats(0, CC_STAT_LEFT), candidate.stats(0, CC_STAT_TOP),
								candidate.stats(0, CC_STAT_WIDTH), candidate.stats(0, CC_STAT_HEIGHT)
								);
						cv::rectangle(vis, boundingBox, cv::Scalar(255, 0, 0), 3);

						{
							sensor_msgs::ImagePtr img =
								cv_bridge::CvImage(
										pcl_conversions::fromPCL(cloud->header),
										"bgr8", vis
										).toImageMsg();

							m_pub_vis.publish(img);

							cv::imwrite((m_currentSavePath / "vis.png").string(), vis);
						}

						poses = findToteGraspPose(
							cloud, normalCloud, toteFrameFloat,
							boundingBox, segmentMask, *maxInfo, &objectStanding);

						timings.addCheckpoint("grasp");
					}

					if(!poses.empty())
					{
						perceptionResult.grasp_poses = poses;
						perceptionResult.grasp_object = maxName;
						perceptionResult.box_coordinates = m_shelfFeatures.getBoxCoordinates();
						perceptionResult.object_standing = objectStanding;
						m_act_perceptionServer->setSucceeded(perceptionResult);
						m_sub_pointCloud.shutdown();

						return;
					}

					// else, continue with next candidate
				}

				break;
			}
		}
	}

	NODELET_INFO_STREAM("Nodelet Done");
}

void PerceptionNodelet::extractRGBimage(const PointCloudXYZRGB::ConstPtr& cloud)
{
	cv::Mat bgrImage = cv::Mat(cloud->height,cloud->width,CV_8UC3);
	for(u_int y = 0; y < cloud->height; ++y)
	{
		for(u_int x =0; x < cloud->width; ++x)
		{
			auto& p = (*cloud)(x,y);
			bgrImage.at<cv::Vec3b>(y,x) = cv::Vec3b(p.b,p.g,p.r) ;
		}
	}
	this->m_featureImages["rgb"] = bgrImage;
}

void clipMatFloat(cv::Mat_<float>& inOut)
{
	inOut.setTo(0, inOut < 0);
	inOut.setTo(1, inOut > 1);
}

void PerceptionNodelet::computeBinMask(const Eigen::Affine3f& shelf2CameraFrame)
{
	cv::Mat binMask;
	m_shelfFeatures.createBinMask(shelf2CameraFrame,binMask);
	this->m_featureImages["binMask"] = binMask;
}

void PerceptionNodelet::computeToteMask(const Eigen::Affine3f& tote2CameraFrame)
{
	cv::Mat toteMask;
	m_toteFeatures.createToteMask(tote2CameraFrame,toteMask);
	this->m_featureImages["binMask"] = toteMask;
}

void PerceptionNodelet::computeDist2shelf()
{
	cv::Mat_<float> dist2Shelf;
	m_shelfFeatures.computeDist2ShelfFeature(dist2Shelf);
	dist2Shelf = dist2Shelf/this->m_dist2shelfNormalizer;
	clipMatFloat(dist2Shelf);
	dist2Shelf *= 255;
	cv::Mat dist2ShelfUchar(dist2Shelf.rows,dist2Shelf.cols,CV_8UC1);
	dist2Shelf.convertTo(dist2ShelfUchar,CV_8UC1);
	this->m_featureImages["dist2shelf"] = dist2ShelfUchar;
}

cv::Mat_<uint8_t> PerceptionNodelet::computeCollisionMask(
	const cv::Mat_<uint8_t>& objectMask,
	const cv::Mat_<uint8_t>& targetMask,
	uint8_t targetId)
{
	auto boxBorder = m_shelfFeatures.getBoxCoordinates();
	constexpr uint8_t LY = 2, MY = 3, LZ = 4, MZ = 5;
	auto maxRows = (int)((boxBorder.boxCoordinates[MZ] -
		boxBorder.boxCoordinates[LZ]) / collisionMaskMeterPerPixel);
	auto maxCols = (int)((boxBorder.boxCoordinates[MY] -
		boxBorder.boxCoordinates[LY]) / collisionMaskMeterPerPixel);

	cv::Mat_<uint8_t> mask( maxRows, maxCols, (uint8_t)255);
	// Note (sebastian): Set an Apron with zeros, since anything
	// beyond the box is an obstacle.
	auto boxCol = m_act_goal->box_col;

	auto sideBarOffset = (int)((0.04f) / collisionMaskMeterPerPixel);
	for(auto row = 0; row < mask.rows; ++row)
	{
		mask(row, 0) = 0;
		mask(row, mask.cols - 1) = 0;
		if(boxCol == 0)
			for(auto col = 0; col < sideBarOffset; ++col)
				mask(row, col) = 0;
		if(boxCol == 2)
			for(auto col = mask.cols - 1; col > (mask.cols - sideBarOffset); --col)
				mask(row, col) = 0;
	}

	double meanTargetDepth = 0.0f;
	int targetPixels = 0;

	for(auto row = 0; row < targetMask.rows; ++row)
	{
		for(auto col = 0; col < targetMask.cols; ++col)
		{
			if(!targetMask(row, col))
				continue;

			const auto p = (*m_cloudInContainer)(col, row);

			if(!std::isfinite(p.x))
				continue;

			meanTargetDepth += p.x;
			++targetPixels;
		}
	}
	meanTargetDepth /= targetPixels;

	for(auto row = 0; row < objectMask.rows; ++row)
	{
		for(auto col = 0; col < objectMask.cols; ++col)
		{
			if(!objectMask(row, col) || (objectMask(row, col) == targetId))
				continue;
			const auto& p = (*m_cloudInContainer)(col, row);
			if(!std::isfinite(p.x))
				continue;

			if(p.x > meanTargetDepth)
				continue;
			auto idX = (int)((boxBorder.boxCoordinates[MY] - p.y) /
				collisionMaskMeterPerPixel);
			auto idY = (int)((boxBorder.boxCoordinates[MZ] - p.z) /
				collisionMaskMeterPerPixel);
			if(idX < maxCols && idX >= 0 && idY < maxRows && idY >= 0)
				mask(idY, idX) = 0;
		}
	}

	return mask;
}

void PerceptionNodelet::computeShelfHeightFeatures()
{
	cv::Mat_<float> height3D,height2D,depth;
	cv::Mat isValidHeight;

	m_shelfFeatures.computeHeightFeatures(
		m_tfCamera2Shelf ,height3D,
		height2D, isValidHeight,depth
	);

	// features  height3D,height2D are floating point Mat and convert them to scale of 0-255
	height3D = height3D / this->m_heightNormalizer;
	clipMatFloat(height3D);
	height3D *= 255;
	cv::Mat height3DUchar(height3D.rows,height3D.cols,CV_8UC1);
	height3D.convertTo(height3DUchar,CV_8UC1);
	this->m_featureImages["height3D"] = height3DUchar;

	height2D = height2D / this->m_heightNormalizer;
	clipMatFloat(height2D);
	height2D *= 255;
	cv::Mat height2DUchar(height2D.rows,height2D.cols,CV_8UC1);
	height2D.convertTo(height2DUchar,CV_8UC1);
	this->m_featureImages["height2D"] = height2DUchar;

	// invert isValidHeiht to get miss3D feature
	cv::Mat miss3D(isValidHeight.rows,isValidHeight.cols,CV_8UC1);
	miss3D.setTo(0, isValidHeight == 1);
	miss3D.setTo(1, isValidHeight == 0);

	this->m_featureImages["miss3D"] = miss3D;

	this->m_featureImages["depth"] = depth;
}

void PerceptionNodelet::computeToteHeightFeatures()
{
	cv::Mat_<float> height3D,depth;
	cv::Mat isValidHeight;

	m_toteFeatures.computeHeightFeatures(
			height3D, isValidHeight, depth
	);

	// features  height3D,height2D are floating point Mat and convert them to scale of 0-255
	height3D = height3D / this->m_heightNormalizer;
	clipMatFloat(height3D);
	height3D *= 255;
	cv::Mat height3DUchar(height3D.rows,height3D.cols,CV_8UC1);
	height3D.convertTo(height3DUchar,CV_8UC1);
	this->m_featureImages["height3D"] = height3DUchar;

	// invert isValidHeiht to get miss3D feature
	cv::Mat miss3D(isValidHeight.rows,isValidHeight.cols,CV_8UC1);
	miss3D.setTo(0, isValidHeight == 1);
	miss3D.setTo(1, isValidHeight == 0);

	this->m_featureImages["miss3D"] = miss3D;

	this->m_featureImages["depth"] = depth;
}

void PerceptionNodelet::computeHHA(const Eigen::Affine3f& cameraPose,const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData)
{
	this->m_featureImages["hha"] = m_shelfFeatures.computeHHA(cameraPose,rawData);
}

void PerceptionNodelet::computeEdgeFeature()
{
	cv::Mat edgeFeature;
	if(m_mode == MODE_SHELF)
	{
		m_shelfFeatures.computeEdge(edgeFeature);
	}
	else
	{
		m_toteFeatures.computeEdgeFeature(edgeFeature);
	}
	this->m_featureImages["edge"] = edgeFeature;
	NODELET_DEBUG_STREAM("  Edge features computed ");
}

}

PLUGINLIB_EXPORT_CLASS(apc_perception::PerceptionNodelet, nodelet::Nodelet);
