// Play back bag file recorded with the aic tool
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

#include <image_geometry/pinhole_camera_model.h>

#include <boost/program_options.hpp>

namespace
{
	constexpr float SCALE = 0.000125f;
}

struct CameraData
{
	sensor_msgs::Image rgb;
	sensor_msgs::Image depth;
	sensor_msgs::Image rotated_rect_color;
	sensor_msgs::CameraInfo camera_info;
	sensor_msgs::CameraInfo rotated_camera_info;

	ros::Publisher pub_rgb;
	ros::Publisher pub_depth;
	ros::Publisher pub_rotated_rect_color;
	ros::Publisher pub_camera_info;
	ros::Publisher pub_rotated_camera_info;

	void advertise(ros::NodeHandle& nh, const std::string& base)
	{
		pub_rgb = nh.advertise<sensor_msgs::Image>(base + "rgb", 5);
		pub_depth = nh.advertise<sensor_msgs::Image>(base + "depth", 5);
		pub_rotated_rect_color = nh.advertise<sensor_msgs::Image>(base + "rotated/image_rect_color", 5);
		pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>(base + "camera_info", 5);
		pub_rotated_camera_info = nh.advertise<sensor_msgs::CameraInfo>(base + "rotated/camera_info", 5);
	}

	void publish(const ros::Time& stamp)
	{
		// Update stamps
		rgb.header.stamp = stamp;
		depth.header.stamp = stamp;
		rotated_rect_color.header.stamp = stamp;
		camera_info.header.stamp = stamp;
		rotated_camera_info.header.stamp = stamp;

		// Publish
		pub_rgb.publish(rgb);
		pub_depth.publish(depth);
		pub_rotated_rect_color.publish(rotated_rect_color);
		pub_camera_info.publish(camera_info);
		pub_rotated_camera_info.publish(rotated_camera_info);
	}
};

CameraData g_camData[2];

void update()
{
	ros::Time now = ros::Time::now();

	// Truncate stamp as PCL does it. This allows us to use exact time sync
	// across topics published from PCL nodes and regular ROS topics.
	now = pcl_conversions::fromPCL(pcl_conversions::toPCL(now));

	for(int i = 0; i < 2; ++i)
		g_camData[i].publish(now);
}

cv::Mat_<uint16_t> nearestNeighbor(const cv::Mat_<uint16_t>& src)
{
	cv::Mat_<uint8_t> invMask = (src == 0);

	std::vector<int> neighborIndices;
	neighborIndices.reserve(src.rows * src.cols);
	for(int y = 0; y < src.rows; ++y)
	{
		for(int x = 0; x < src.cols; ++x)
		{
			if(invMask(y,x) == 0)
			{
				neighborIndices.push_back(y * src.cols + x);
			}
		}
	}

	cv::Mat_<int> nearestNeighbors;
	cv::Mat_<float> distances;
	cv::distanceTransform(invMask, distances, nearestNeighbors, CV_DIST_L2, 5, cv::DIST_LABEL_PIXEL);

	cv::Mat_<uint16_t> depthOut;
	src.copyTo(depthOut);

	for(int y = 0; y < src.rows; ++y)
	{
		for(int x = 0; x < src.cols; ++x)
		{
			if(invMask(y,x) != 0 && distances(y,x) < 3)
			{
				depthOut(y,x) = depthOut[0][neighborIndices[nearestNeighbors(y,x)-1]];
			}
		}
	}

	return depthOut;
}

int main(int argc, char** argv)
{
	namespace po = boost::program_options;

	ros::init(argc, argv, "play_bag");

	po::options_description desc("Options");
	desc.add_options()
		("help", "produce help message")
		("bag", po::value<std::string>(), "bag file")
		("dump", "dump raw data")
		("once", "Publish once and exit")
	;

	po::positional_options_description p;
	p.add("bag", 1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
	po::notify(vm);

	if(vm.count("help"))
	{
		std::cerr << desc << "\n";
		return 1;
	}

	if(!vm.count("bag"))
	{
		std::cerr << "bag argument is required!\n";
		return 1;
	}

	rosbag::Bag bag(vm["bag"].as<std::string>(), rosbag::bagmode::Read);

	rosbag::View view(bag);

	for(auto& record : view)
	{
		std::string topic = record.getTopic();

		CameraData* camData = 0;

		if(topic.substr(0, 20) == "/camera_stereo/cam1/")
			camData = &g_camData[0];
		else if(topic.substr(0, 20) == "/camera_stereo/cam2/")
			camData = &g_camData[1];
		else
		{
			ROS_WARN("Unknown topic '%s' in bag", topic.c_str());
			continue;
		}

		std::string subtopic = topic.substr(20);

		if(subtopic == "rgb")
			camData->rgb = *record.instantiate<sensor_msgs::Image>();
		else if(subtopic == "depth")
			camData->depth = *record.instantiate<sensor_msgs::Image>();
		else if(subtopic == "rotated/image_rect_color")
			camData->rotated_rect_color = *record.instantiate<sensor_msgs::Image>();
		else if(subtopic == "camera_info")
			camData->camera_info = *record.instantiate<sensor_msgs::CameraInfo>();
		else if(subtopic == "rotated/camera_info")
			camData->rotated_camera_info = *record.instantiate<sensor_msgs::CameraInfo>();
	}

	if(vm.count("dump"))
	{
		ROS_INFO("Dumping raw depth and RGB image");

		auto depth = cv_bridge::toCvCopy(g_camData[1].depth);
		cv::imwrite("/tmp/cam2_depth.png", depth->image);
		cv::imwrite("/tmp/cam2_filled.png", 10.0 * nearestNeighbor(depth->image));
		ROS_INFO("Depth: %s", depth->encoding.c_str());

		auto depth1 = cv_bridge::toCvCopy(g_camData[0].depth);
		cv::imwrite("/tmp/cam1_depth.png", depth1->image);
		cv::imwrite("/tmp/cam1_filled.png", 10.0 * nearestNeighbor(depth1->image));

		auto rgb = cv_bridge::toCvCopy(g_camData[1].rgb, "bgr8");
		cv::imwrite("/tmp/cam2_rgb.png", rgb->image);

		ROS_INFO("Writing input point cloud");
		pcl::PointCloud<pcl::PointXYZRGB> cloud(depth->image.cols, depth->image.rows);

		image_geometry::PinholeCameraModel model;
		model.fromCameraInfo(g_camData[1].camera_info);

		for(int y = 0; y < depth->image.rows; ++y)
		{
			for(int x = 0; x < depth->image.cols; ++x)
			{
				pcl::PointXYZRGB& p = cloud(x,y);
				cv::Vec3b bgr = rgb->image.at<cv::Vec3b>(y,x);
				p.r = bgr[2];
				p.g = bgr[1];
				p.b = bgr[0];

				uint16_t depthVal = depth->image.at<uint16_t>(y,x);
				if(depthVal == 0)
				{
					p.x = p.y = p.z = NAN;
				}
				else
				{
					auto ray = model.projectPixelTo3dRay(cv::Point2d(x,y));
					float d = depthVal * SCALE;

					p.x = ray.x * d;
					p.y = ray.y * d;
					p.z = d;
				}
			}
		}

		pcl::io::savePCDFileBinary("/tmp/cam2.pcd", cloud);

		ROS_INFO("Finished.");
	}

	ros::NodeHandle nh;

	for(int cam = 0; cam < 2; ++cam)
	{
		char base[256];
		snprintf(base, sizeof(base), "/camera_stereo/cam%d/", cam+1);

		g_camData[cam].advertise(nh, base);
	}

	if(vm.count("once"))
	{
		for(int i = 0; i < 20; ++i)
		{
			ros::spinOnce();
			usleep(100*1000);
		}

		update();

		for(int i = 0; i < 20; ++i)
		{
			ros::spinOnce();
			usleep(100*1000);
		}
	}
	else
	{
		ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&update));
		timer.start();

		ros::spin();
	}

	return 0;
}
