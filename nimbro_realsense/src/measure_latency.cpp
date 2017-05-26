// Measure latency on a pointcloud topic (header stamp vs. current time)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <sensor_msgs/PointCloud2.h>

void handleCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	ROS_INFO("Latency: %5.3fs",
		(ros::Time::now() - msg->header.stamp).toSec()
	);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "measure_latency");

	ros::NodeHandle nh("~");
	ros::Subscriber sub = nh.subscribe("input", 1, &handleCloud);

	ros::spin();

	return 0;
}
