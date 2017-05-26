// Capture data from a point cloud topic
// Author: Arul Selvam Periyasamy
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <camera_calibration_parsers/parse.h>

#include <actionlib/client/simple_action_client.h>

#include <nimbro_keyframe_server/PlayMotionAction.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rosbag/bag.h>

#include <string>
#include <iostream>
#include <memory>

#include <termios.h>

bool g_canWrite = false;
bool g_written = false;
boost::mutex g_lock;
boost::condition_variable g_cond;

std::string g_imgpath;
int g_imgCount = 0;
int g_binX;
int g_binY;

sensor_msgs::CameraInfoConstPtr g_cam1_info;
sensor_msgs::CameraInfoConstPtr g_cam1_rotated_info;
sensor_msgs::CameraInfoConstPtr g_cam2_info;
sensor_msgs::CameraInfoConstPtr g_cam2_rotated_info;

sensor_msgs::CameraInfo restamp(const sensor_msgs::CameraInfo& input, const ros::Time& stamp)
{
	sensor_msgs::CameraInfo ret(input);
	ret.header.stamp = stamp;
	return ret;
}

void handleData(
	const sensor_msgs::ImageConstPtr& img_cam1_rgb,
	const sensor_msgs::ImageConstPtr& img_cam1_depth,
	const sensor_msgs::ImageConstPtr& img_cam1_rotated_rgb,

	const sensor_msgs::ImageConstPtr& img_cam2_rgb,
	const sensor_msgs::ImageConstPtr& img_cam2_depth,
	const sensor_msgs::ImageConstPtr& img_cam2_rotated_rgb,

	const sensor_msgs::PointCloud2ConstPtr& cloud_msg
)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::fromROSMsg(*cloud_msg, cloud);

	{
		boost::mutex::scoped_lock lock(g_lock);
		if(!g_canWrite)
			return;
	}

	if(!g_cam1_info || !g_cam1_rotated_info || !g_cam2_info || !g_cam2_rotated_info)
	{
		ROS_INFO("camera info missing...");
		return;
	}

	std::stringstream ss;
	ss << std::setw(3) << std::setfill('0') << g_imgCount++;
	boost::filesystem::path dirPath(g_imgpath+"image" + ss.str());

	if(!boost::filesystem::create_directories(dirPath))
	{
		ROS_ERROR("Could not create output directory '%s'", dirPath.c_str());
		return;
	}

	ROS_INFO("Writing frame to '%s'", dirPath.c_str());

	int binX = g_binX;
	int binY = g_binY;

	// Signal other thread that we got the data
	{
		boost::mutex::scoped_lock lock(g_lock);

		g_canWrite = false;
		g_written = true;
		g_cond.notify_all();
	}

	pcl::io::savePCDFileBinaryCompressed((dirPath / "frame.pcd").string(), cloud);
	camera_calibration_parsers::writeCalibration((dirPath / "camera.ini").string(), "camera", *g_cam2_info);

	{
		std::ofstream context((dirPath / "box_index.txt").string());
		context << binY << ' ' << binX << '\n';
	}

	// Also extract an RGB image for convenience
	{
		cv::Mat_<cv::Vec3b> rgb(cloud.height, cloud.width);
		unsigned int cloudIdx = 0;
		for(int y = 0; y < rgb.rows; ++y)
		{
			for(int x = 0; x < rgb.cols; ++x)
			{
				const auto& p = cloud[cloudIdx];
				rgb(y,x) = cv::Vec3b(p.b, p.g, p.r);
				cloudIdx++;
			}
		}

		cv::imwrite((dirPath / "rgb.png").string(), rgb);
	}

	// Write bag file with raw data
	{
		rosbag::Bag bag((dirPath / "capture.bag").string(), rosbag::bagmode::Write);
		bag.setCompression(rosbag::compression::BZ2);

		ros::Time stamp = img_cam1_rgb->header.stamp;

		bag.write("/camera_stereo/cam1/rgb", stamp, img_cam1_rgb);
		bag.write("/camera_stereo/cam1/depth", stamp, img_cam1_depth);
		bag.write("/camera_stereo/cam1/rotated/image_rect_color", stamp, img_cam1_rotated_rgb);
		bag.write("/camera_stereo/cam1/camera_info", stamp, restamp(*g_cam1_info, stamp));
		bag.write("/camera_stereo/cam1/rotated/camera_info", stamp, restamp(*g_cam1_rotated_info, stamp));

		bag.write("/camera_stereo/cam2/rgb", stamp, img_cam2_rgb);
		bag.write("/camera_stereo/cam2/depth", stamp, img_cam2_depth);
		bag.write("/camera_stereo/cam2/rotated/image_rect_color", stamp, img_cam2_rotated_rgb);
		bag.write("/camera_stereo/cam2/camera_info", stamp, restamp(*g_cam2_info, stamp));
		bag.write("/camera_stereo/cam2/rotated/camera_info", stamp, restamp(*g_cam2_rotated_info, stamp));

		bag.close();
	}

	ROS_INFO("done.");
}

void handleCam1Info(const sensor_msgs::CameraInfoConstPtr& info)
{
	g_cam1_info = info;
}

void handleCam1RotatedInfo(const sensor_msgs::CameraInfoConstPtr& info)
{
	g_cam1_rotated_info = info;
}

void handleCam2Info(const sensor_msgs::CameraInfoConstPtr& info)
{
	g_cam2_info = info;
}

void handleCam2RotatedInfo(const sensor_msgs::CameraInfoConstPtr& info)
{
	g_cam2_rotated_info = info;
}

void moveTo(actionlib::SimpleActionClient<nimbro_keyframe_server::PlayMotionAction>* ac, const std::string& motion)
{
	nimbro_keyframe_server::PlayMotionGoal goal;
	goal.motion_name = motion;
	goal.use_existing_motion = true;

	ROS_INFO("Requesting arm movement to '%s'", goal.motion_name.c_str());
	ac->sendGoal(goal);

	bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));

	if(!finished_before_timeout)
	{
		ROS_ERROR("timeout on arm movement. Exiting :-(");
		throw std::runtime_error("timeout");
	}

	actionlib::SimpleClientGoalState state = ac->getState();
	if(state != state.SUCCEEDED)
	{
		ROS_ERROR("Unexpected action goal state: '%s'", state.toString().c_str());
		throw std::runtime_error("unexpected action state");
	}

	if(!ac->getResult()->success)
	{
		ROS_ERROR("Something went wrong during playback");
		throw std::runtime_error("unexpected action state");
	}

	ROS_INFO("Movement finished.");
}

void keyboardCallback()
{
	actionlib::SimpleActionClient<nimbro_keyframe_server::PlayMotionAction> ac("/ros_player/play_motion", true);

	ROS_INFO("Waiting for action server");
	ac.waitForServer();
	ROS_INFO("Got it!");

	for(int row = 0; row < 4; ++row)
	{
		for(int col = 0; col < 3; ++col)
		{
			g_binX = col;
			g_binY = row;

			char box = 'A' + (row * 3) + col;

			// Move arm to box
			moveTo(&ac, std::string("capture_") + box);

			// Wait for image to stabilize
			sleep(7);

			// Request snapshot
			{
				boost::mutex::scoped_lock lock(g_lock);
				g_written = false;
				g_canWrite = true;

				while(!g_written)
					g_cond.wait(lock);
			}
		}
	}
/*
	ROS_INFO("moving into start position...");

	moveTo(&ac, "capture_K");
	moveTo(&ac, "capture_E");
	moveTo(&ac, "capture_away");

	ROS_INFO("finished!");
	ros::requestShutdown();

	printf("Press enter to move into the capture position\n");
	getchar();

	while (true)
	{
		moveTo(&ac, "capture_tote");

		printf("Ready. Press enter to trigger snapshot!\n");
		getchar();
		sleep(7);

		// Request snapshot
		{
			boost::mutex::scoped_lock lock(g_lock);
			g_binX = 10;
			g_written = false;
			g_canWrite = true;

			while(!g_written)
				g_cond.wait(lock);
		}

		moveTo(&ac, "capture_tote_angle");
		sleep(7);

		// Request snapshot
		{
			boost::mutex::scoped_lock lock(g_lock);
			g_binX = 11;
			g_written = false;
			g_canWrite = true;

			while(!g_written)
				g_cond.wait(lock);
		}
	}*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ImageCapture");
	ros::NodeHandle nh("~");

	if(!nh.getParam("bin_x", g_binX) || !nh.getParam("bin_y", g_binY))
	{
		ROS_ERROR("need parameters bin_x, bin_y");
		return 1;
	}
	ROS_INFO("Operating on bin y=%d, x=%d", g_binY, g_binX);

	if(!nh.getParam("save_path", g_imgpath))
	{
		ROS_ERROR("Parameter ~save_path not specified");
		return 1;
	}

	// check if last character of image save path is '/' , if not add it
	if(g_imgpath[g_imgpath.length()-1] != '/')
		g_imgpath.append( "/");

	ROS_INFO("Saving images to '%s'", g_imgpath.c_str());
	ROS_INFO("Waiting for images...");

	image_transport::ImageTransport it(nh);

	image_transport::SubscriberFilter sub_cam1_rgb(it, "/camera_stereo/cam1/rgb", 10);
	image_transport::SubscriberFilter sub_cam1_depth(it, "/camera_stereo/cam1/depth", 10);
	image_transport::SubscriberFilter sub_cam1_rotated_rgb(it, "/camera_stereo/cam1/rotated/image_rect_color", 10);

	image_transport::SubscriberFilter sub_cam2_rgb(it, "/camera_stereo/cam2/rgb", 10);
	image_transport::SubscriberFilter sub_cam2_depth(it, "/camera_stereo/cam2/depth", 10);
	image_transport::SubscriberFilter sub_cam2_rotated_rgb(it, "/camera_stereo/cam2/rotated/image_rect_color", 10);

	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(nh, "/camera_filler/output", 2);

	typedef message_filters::sync_policies::ApproximateTime<
		sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
		sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
		sensor_msgs::PointCloud2
	> MySyncPolicy;

	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(200),
		sub_cam1_rgb, sub_cam1_depth, sub_cam1_rotated_rgb,
		sub_cam2_rgb, sub_cam2_depth, sub_cam2_rotated_rgb,
		sub_cloud
	);

	sync.registerCallback(&handleData);

	ros::Subscriber info1_sub = nh.subscribe("/camera_stereo/cam1/camera_info", 1, &handleCam1Info);
	ros::Subscriber info1rot_sub = nh.subscribe("/camera_stereo/cam1/rotated/camera_info", 1, &handleCam1RotatedInfo);
	ros::Subscriber info2_sub = nh.subscribe("/camera_stereo/cam2/camera_info", 1, &handleCam2Info);
	ros::Subscriber info2rot_sub = nh.subscribe("/camera_stereo/cam2/rotated/camera_info", 1, &handleCam2RotatedInfo);

	boost::thread keyboardThread(keyboardCallback);

	ros::spin();

	keyboardThread.join();

	return 0;
}
