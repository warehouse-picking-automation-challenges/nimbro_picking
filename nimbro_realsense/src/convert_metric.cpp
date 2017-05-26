// Convert SR300 measurements in uint16_t to metric float
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "convert_metric.h"

#include <ros/node_handle.h>

#include <sensor_msgs/image_encodings.h>

#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>

namespace
{
	constexpr float SCALE = 0.000125f;
}

namespace nimbro_realsense
{

void ConvertMetric::onInit()
{
	auto& nh = getPrivateNodeHandle();

	auto cb = boost::bind(&ConvertMetric::checkSubscriber, this);
	m_pub_image = nh.advertise<sensor_msgs::Image>("output", 1, cb, cb);

	// This flag allows to truncate the output stamp as PCL does - this
	// allows exact synchronization to point cloud topics published by PCL code
	nh.param("truncate_stamp_pcl", m_truncateStampPCL, false);

	checkSubscriber();
}

void ConvertMetric::checkSubscriber()
{
	bool needed = m_pub_image.getNumSubscribers();

	if(needed && !m_active)
	{
		m_sub_image = getPrivateNodeHandle().subscribe("input", 1, &ConvertMetric::handleImage, this);
		NODELET_INFO("subscribed to '%s'", m_sub_image.getTopic().c_str());
		m_active = true;
	}
	else if(!needed && m_active)
	{
		m_sub_image.shutdown();
		m_active = false;
	}
}

void ConvertMetric::handleImage(const sensor_msgs::ImageConstPtr& img)
{
	if(img->encoding != sensor_msgs::image_encodings::MONO16)
	{
		NODELET_ERROR("Unknown image encoding: '%s', expected MONO16", img->encoding.c_str());
		return;
	}

	sensor_msgs::ImagePtr out(new sensor_msgs::Image);

	out->header = img->header;

	if(m_truncateStampPCL)
		out->header.stamp = pcl_conversions::fromPCL(pcl_conversions::toPCL(img->header.stamp));

	out->width = img->width;
	out->height = img->height;
	out->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	out->step = out->width * sizeof(float);

	out->data.resize(out->height * out->step);

	for(unsigned int y = 0; y < img->height; ++y)
	{
		const uint16_t* input = reinterpret_cast<const uint16_t*>(img->data.data() + img->step * y);
		float* output = reinterpret_cast<float*>(out->data.data() + out->step * y);

		for(unsigned int x = 0; x < img->width; ++x)
		{
			if(*input == 0)
				*output = NAN;
			else
				*output = SCALE * *input;

			input++;
			output++;
		}
	}

	m_pub_image.publish(out);
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_realsense::ConvertMetric, nodelet::Nodelet)
