// Subclass of ros_control_player with APC specific behaviour
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>

#include <ros_control_player/ros_control_player.h>

#include <apc_objects/apc_objects.h>

#include <tf/transform_listener.h>

#include <tf_conversions/tf_eigen.h>

#include <ros/init.h>

static const Eigen::Affine3d expectedShelfPose(
	Eigen::Translation3d(apc_objects::distRobot2Shelf, 0.0, 0.0)
);

class APCRosPlayer : public nimbro_keyframe_server::ROSControlPlayer
{
	using nimbro_keyframe_server::ROSControlPlayer::ROSControlPlayer;
private:
	void modifyMotion(nimbro_keyframe_server::Motion::Ptr motion) override;

	tf::TransformListener m_tf;
};

void APCRosPlayer::modifyMotion(nimbro_keyframe_server::Motion::Ptr motion)
{
	ROS_INFO("motion:\n%s", motion->serializeToYAML().c_str());

	Eigen::Affine3d shelfPose = expectedShelfPose;
	try
	{
		tf::StampedTransform transform;
		m_tf.lookupTransform("world", "registration_shelf", ros::Time(0), transform);

		tf::transformTFToEigen(transform, shelfPose);
	}
	catch(tf::TransformException& e)
	{
		ROS_ERROR("Could not get shelf registration transform: '%s'", e.what());
		ROS_ERROR("Using default...");
	}

	ROS_INFO_STREAM("shelf pose:\n" << shelfPose.matrix() << "\nexpected:\n" << expectedShelfPose.matrix());

	for(auto& keyframe : *motion)
	{
		std::string label = keyframe.label();

		if(label.find("[shelf]") != label.npos)
		{
			for(auto& group : keyframe.jointGroups())
			{
				if(group.second.interpolationSpace() != keyframe.IS_CARTESIAN)
					continue;

				Eigen::Affine3d old = group.second.state();
				keyframe.setState(group.first,
					shelfPose * expectedShelfPose.inverse() * old
				);
			}
		}
	}

	ROS_INFO("after adaption:\n%s", motion->serializeToYAML().c_str());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "apc_ros_player");

	ros::NodeHandle nh("~");

	APCRosPlayer p;
	if(!p.init())
	{
		ROS_ERROR("Failed to initialize APCRosPlayer. Abort!");
		return 1;
	}

	ros::spin();
	return 0;
}

