// APC arm motion generator
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef APC_ARM_H
#define APC_ARM_H

#include <robotcontrol/motionmodule.h>
#include <robotcontrol/model/joint.h>
#include <robotcontrol/model/robotmodel.h>
#include <robotcontrol/FadeTorqueAction.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <nimbro_keyframe_server/keyframe_server.h>
#include <nimbro_keyframe_server/motion.h>
#include <nimbro_keyframe_server/moveit_player.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <plot_msgs/Plot.h>

#include <config_server/parameter.h>

#include "body_part.h"

namespace apc_motion
{

class APCMotion : public robotcontrol::MotionModule
{
public:
	APCMotion();
	virtual ~APCMotion();

	virtual bool init(robotcontrol::RobotModel* model);

	virtual bool isTriggered();
	virtual void step();

	virtual void handleEmergencyStop() override;

private:
	bool playMotion(const nimbro_keyframe_server::Motion::Ptr& motion);

	void playMotionGoal();

	std::vector<std::string> getJointSpaceJoints(const nimbro_keyframe_server::KeyFrame& frame);
	void handleStartKeyframe(const nimbro_keyframe_server::KeyFrame& frame);
	void handleEndKeyframe(const nimbro_keyframe_server::KeyFrame& frame);

	robot_model_loader::RobotModelLoaderPtr m_moveitLoader;
	robot_model::RobotModelPtr m_moveitModel;
	robot_state::RobotStatePtr m_moveitState;

	nimbro_keyframe_server::KeyFrameServer m_keyframeServer;
	boost::shared_ptr<nimbro_keyframe_server::MoveItPlayer> m_player;

	std::vector<BodyPart::Ptr> m_parts;

	plot_msgs::Plot m_plot;
	ros::Publisher m_pub_plot;

	ros::Subscriber m_sub_armControl;

	config_server::Parameter<float> m_armControl_lin_vel;
	config_server::Parameter<float> m_armControl_rot_vel;

	robotcontrol::RobotModel* m_model;
	robotcontrol::RobotModel::State m_state_unsafe_connection;
};

}

#endif
