// Convert SR300 measurements in uint16_t to metric float
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CONVERT_METRIC_H
#define CONVERT_METRIC_H

#include <nodelet/nodelet.h>

#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <sensor_msgs/Image.h>

namespace nimbro_realsense
{

class ConvertMetric : public nodelet::Nodelet
{
public:
	void onInit() override;
private:
	void handleImage(const sensor_msgs::ImageConstPtr& img);
	void checkSubscriber();

	ros::Subscriber m_sub_image;
	ros::Publisher m_pub_image;

	bool m_truncateStampPCL;
	bool m_active = false;
};

}

#endif
