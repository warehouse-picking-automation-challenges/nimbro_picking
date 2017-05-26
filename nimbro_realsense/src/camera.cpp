// Single camera ROS publisher
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "camera.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

namespace nimbro_realsense
{

Camera::Camera(const ros::NodeHandle& nh, rs::device* device, const std::string& calibURL, const std::string& rotatedCalibURL, bool upper)
 : m_nh(nh)
 , m_device(device)
 , m_upper(upper)
 , m_infoManager(nh, device->get_serial(), calibURL)
 , m_rotated_infoManager(ros::NodeHandle(nh, "rotated"), device->get_serial(), rotatedCalibURL)
{
	ROS_INFO("Connected device: '%s', serial %s, firmware %s",
		m_device->get_name(),
		m_device->get_serial(),
		m_device->get_firmware_version()
	);

	nh.param("frame_id", m_frame_id, std::string("realsense_rgb_optical"));

	// TODO: Configuration!
	m_device->enable_stream(rs::stream::color, 1920, 1080, rs::format::rgb8, 0);
	m_device->enable_stream(rs::stream::depth, rs::preset::best_quality);

	// Prepare publication
	auto cb = boost::bind(boost::ref(this->subscriptionChanged));

	m_it.reset(new image_transport::ImageTransport(m_nh));
	m_pub_color = m_it->advertiseCamera("rgb", 1, cb, cb);
	m_pub_depth = m_it->advertise("depth", 1, cb, cb);
	m_pub_pointcloud = m_nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("cloud", 1,
		cb, cb
	);

	m_rotated_it.reset(new image_transport::ImageTransport(ros::NodeHandle(m_nh, "rotated")));
	m_pub_rotated_color = m_rotated_it->advertiseCamera("image_raw", 1, cb, cb);
}

Camera::~Camera()
{
}

bool Camera::hasSubscribers()
{
	return m_pub_color.getNumSubscribers() != 0
	    || m_pub_depth.getNumSubscribers() != 0
	    || m_pub_pointcloud.getNumSubscribers() != 0;
}

void Camera::start()
{
	m_device->start();
}

void Camera::stop()
{
	m_device->stop();
}

void Camera::capture(ros::Time stamp)
{
	auto colorIntrinsics = m_device->get_stream_intrinsics(rs::stream::rectified_color);
	auto depthIntrinsics = m_device->get_stream_intrinsics(rs::stream::depth_aligned_to_rectified_color);

	auto scale = m_device->get_depth_scale();
	ROS_INFO_ONCE("Depth scale: %f", scale);

	m_device->wait_for_frames();

	if(stamp == ros::Time(0))
		stamp = ros::Time::now();

	const uint8_t* colorData = reinterpret_cast<const uint8_t*>(
		m_device->get_frame_data(rs::stream::rectified_color)
	);

	const uint16_t* depthData = reinterpret_cast<const uint16_t*>(
		m_device->get_frame_data(rs::stream::depth_aligned_to_rectified_color)
	);

	if(true) // always create image
	{
		sensor_msgs::ImagePtr colorImg(new sensor_msgs::Image);
		colorImg->header.stamp = stamp;
		colorImg->header.frame_id = m_frame_id;

		colorImg->encoding = sensor_msgs::image_encodings::RGB8;
		colorImg->data.resize(colorIntrinsics.width * colorIntrinsics.height * 3);
		memcpy(colorImg->data.data(), colorData, colorImg->data.size());

		colorImg->width = colorIntrinsics.width;
		colorImg->height = colorIntrinsics.height;
		colorImg->step = colorImg->width * 3;

		sensor_msgs::CameraInfoPtr colorInfo(new sensor_msgs::CameraInfo);
		colorInfo->header = colorImg->header;
		colorInfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		colorInfo->width = colorIntrinsics.width;
		colorInfo->height = colorIntrinsics.height;
		colorInfo->D.resize(5, 0.0);

		colorInfo->K.assign(0.0);
		colorInfo->K[0] = colorIntrinsics.fx;
		colorInfo->K[4] = colorIntrinsics.fy;
		colorInfo->K[2] = colorIntrinsics.ppx;
		colorInfo->K[5] = colorIntrinsics.ppy;
		colorInfo->K[8] = 1.0;

		colorInfo->R.assign(0.0);
		colorInfo->R[0] = colorInfo->R[4] = colorInfo->R[8] = 1.0;

		colorInfo->P.assign(0.0);
		colorInfo->P[0] = colorIntrinsics.fx;
		colorInfo->P[5] = colorIntrinsics.fy;
		colorInfo->P[2] = colorInfo->K[2]; // cx
		colorInfo->P[6] = colorInfo->K[5]; // cy
		colorInfo->P[10] = 1.0;

		m_lastImage = colorImg;
		m_pub_color.publish(colorImg, colorInfo);
	}

	if(m_pub_depth.getNumSubscribers())
	{
		sensor_msgs::ImagePtr depthImg(new sensor_msgs::Image);
		depthImg->header.stamp = stamp;
		depthImg->header.frame_id = m_frame_id;

		depthImg->encoding = sensor_msgs::image_encodings::MONO16;
		depthImg->data.resize(colorIntrinsics.width * colorIntrinsics.height * 2);
		memcpy(depthImg->data.data(), depthData, depthImg->data.size());

		depthImg->width = colorIntrinsics.width;
		depthImg->height = colorIntrinsics.height;
		depthImg->step = depthImg->width * 2;

		m_pub_depth.publish(depthImg);
	}

	if(m_pub_pointcloud.getNumSubscribers())
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>(colorIntrinsics.width, colorIntrinsics.height)
		);

		for(int y = 0; y < colorIntrinsics.height; ++y)
		{
			for(int x = 0; x < colorIntrinsics.width; ++x)
			{
				const uint8_t* color = colorData + (y * colorIntrinsics.width + x) * 3;
				(*cloud)[y * colorIntrinsics.width + x].r = color[0];
				(*cloud)[y * colorIntrinsics.width + x].g = color[1];
				(*cloud)[y * colorIntrinsics.width + x].b = color[2];
				(*cloud)[y * colorIntrinsics.width + x].z = NAN;
			}
		}

		for(int y = 0; y < depthIntrinsics.height; ++y)
		{
			for(int x = 0; x < depthIntrinsics.width; ++x)
			{
				uint16_t depth_value = depthData[y * depthIntrinsics.width + x];
				float depth_in_meters = depth_value * scale;

				if(depth_value == 0)
					continue;

				rs::float2 depth_pixel = {(float)x, (float)y};
				rs::float3 color_point = depthIntrinsics.deproject(depth_pixel, depth_in_meters);

				pcl::PointXYZRGB& p = (*cloud)[y * colorIntrinsics.width + x];
				p.x = color_point.x;
				p.y = color_point.y;
				p.z = color_point.z;
			}
		}

		cloud->header.frame_id = m_frame_id;
		cloud->header.stamp = pcl_conversions::toPCL(stamp);

		m_pub_pointcloud.publish(cloud);
	}

	if(m_pub_rotated_color.getNumSubscribers())
	{
		sensor_msgs::ImagePtr output(new sensor_msgs::Image);

		output->header.stamp = stamp;
		output->header.frame_id = m_frame_id + "_rotated";
		output->width = m_lastImage->height;
		output->height = m_lastImage->width;

		output->encoding = m_lastImage->encoding;
		output->step = output->width * 3;

		output->data.resize(m_lastImage->data.size());

		for(unsigned int y = 0; y < output->height; ++y)
		{
			for(unsigned int x = 0; x < output->width; ++x)
			{

				int inputIdx;
				int outputIdx = 3*(output->width * y + x);

				if(!m_upper)
				{
					inputIdx = 3*(m_lastImage->width * x + (m_lastImage->width - 1 - y));
				}
				else
				{
					inputIdx = 3*(m_lastImage->width * (m_lastImage->height - 1 - x) + y);
				}

				for(int i = 0; i < 3; ++i)
					output->data[outputIdx + i] = m_lastImage->data[inputIdx + i];
			}
		}

		sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo(m_rotated_infoManager.getCameraInfo()));
		camInfo->header = output->header;

		m_pub_rotated_color.publish(output, camInfo);
	}
}

void Camera::setLaserEnabled(bool enabled)
{
	m_device->set_option(rs::option::f200_laser_power, enabled ? 1.0 : 0.0);
}

void Camera::skipFrame()
{
	m_device->wait_for_frames();
}

}
