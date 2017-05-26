#include <apc_capture/box_registration.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Float32.h>
#include <apc_objects/apc_objects.h>
namespace apc_object_perception
{
	BoxRegistration::BoxRegistration() {}
	BoxRegistration::~BoxRegistration() {}

	template<typename PointT>
	void loadPointcloudXYZRGB(std::string pcdfile,pcl::PointCloud<PointT>& cloud )
	{
		if (pcl::io::loadPCDFile<PointT>(pcdfile,cloud) == -1)
		{
			ROS_ERROR("Failed to read the PCL file");
		} else
		{
			ROS_INFO_STREAM("The PCD file loaded");
		}
	}

	void BoxRegistration::onInit()
	{
		m_nh                    = getPrivateNodeHandle();
		m_sub_pointCloud        = m_nh.subscribe("/camera/depth/points",1,&BoxRegistration::cloudSubCallBack,this); // check the queue size here //also check for the topic /cloud_pcd ***check***
		m_rosPackagePath        = ros::package::getPath("apc_capture");
		m_imagePath             = m_rosPackagePath + "/resource/pcd/";
		m_imageName             = "shelf.pcd";

		m_pub_Boxmsg            = m_nh.advertise<apc_capture::BoxCoordinates>("/box_Coordinates",1);

		//m_cloudfromSensorXYZRGB = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>());
		m_cloudfromSensorXYZ    = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());
		m_cloudfromMesh  		= boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());
		m_tf.reset(new tf::TransformListener(m_nh));

		// load the template
		loadPointcloudXYZRGB(m_imagePath+m_imageName,*m_cloudfromMesh);

		// translate the template by apc_objects::distRobot2Shelf
		Eigen::Affine3f translation(Eigen::Translation3f(apc_objects::distRobot2Shelf,0,0));
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*m_cloudfromMesh,*cloudTemp,translation.matrix());

		m_cloudfromMesh=cloudTemp;

		m_features.setMeshCloud(m_cloudfromMesh);

		m_nh.getParam("/fakemode",m_fakeMode);
		Eigen::Vector2i boxIndices(1,1);
		m_features.setBoxIndices(boxIndices);
	}

	void convertpclXYZRGB2XYZ(pcl::PointCloud<pcl::PointXYZRGB>& cloudXYZRGB , pcl::PointCloud<pcl::PointXYZ>& cloudXYZ)
	{
		for (size_t i = 0; i < cloudXYZRGB.points.size(); i++)
		{
			cloudXYZ.points[i].x = cloudXYZRGB.points[i].x;
			cloudXYZ.points[i].y = cloudXYZRGB.points[i].y;
			cloudXYZ.points[i].z = cloudXYZRGB.points[i].z;
		}
	}

	void BoxRegistration::cloudSubCallBack(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
	{
		m_boxmsg.header.stamp         = pcl_conversions::fromPCL(msg->header).stamp;
		m_boxmsg.header.frame_id 	  = "world";
		m_cloudfromSensorXYZ 		  = msg;
		doRegistration();
	}

	void BoxRegistration::doRegistration()
	{

		/*
		Eigen::Affine3f transform;  // ***** this will be replaced by some transform based on TF *****
		transform = Eigen::Translation3f(-0.6, 0, 1.4) *
					Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitZ()) *
					Eigen::AngleAxisf(250.0 * M_PI / 180.0, Eigen::Vector3f::UnitX());

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*m_cloudformSensorXYZ,*cloudTemp,transform);
		m_cloudformSensorXYZ = cloudTemp; */   //Not need for the clouds from sensors

		Eigen::Matrix4f initialguess;

		if(!m_tf->waitForTransform("world", m_cloudfromSensorXYZ->header.frame_id,  pcl_conversions::fromPCL(m_cloudfromSensorXYZ->header.stamp), ros::Duration(1.0)))
		{
			NODELET_ERROR(" The transform wait from cloud to world timeout ");
			return;
		}

		tf::StampedTransform tfTrans;
		try
		{
			m_tf->lookupTransform("world", m_cloudfromSensorXYZ->header.frame_id,
								  pcl_conversions::fromPCL(m_cloudfromSensorXYZ->header.stamp), tfTrans);
		}
		catch(tf::TransformException& e)
		{
			NODELET_ERROR("Could not get input transform: %s", e.what());
			return;
		}

		// transform point cloud to the world coordinate
		Eigen::Affine3d transformSensorInWorldFrame;  // Eigen::Affine3d
		tf::transformTFToEigen(tfTrans, transformSensorInWorldFrame);
		Eigen::Affine3f transformF = transformSensorInWorldFrame.cast<float>();

		Eigen::Affine3f shelfFrame;
		tf::Transform tfShelfFrame ;
		cv::Mat shelfMask;
		m_features.doRegistration(m_cloudfromSensorXYZ,transformF,shelfMask,&shelfFrame);

		NODELET_INFO_STREAM("Got shelf frame:\n" << shelfFrame.matrix());

		tf::transformEigenToTF(shelfFrame.cast<double>(), tfShelfFrame);

		m_tf_broadcaster.sendTransform(tf::StampedTransform(
			tfShelfFrame, pcl_conversions::fromPCL(m_cloudfromSensorXYZ->header.stamp),
			"world", "perception_shelf"));
		NODELET_INFO("world 2 shelf published" );
	}

} // end namespace apc_object_perception


PLUGINLIB_EXPORT_CLASS(apc_object_perception::BoxRegistration, nodelet::Nodelet);



