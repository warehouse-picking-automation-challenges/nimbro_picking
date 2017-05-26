// KeyFrame editor for rviz
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CENTAURO_KEYFRAME_SERVER_KEYFRAME_EDITOR_H
#define CENTAURO_KEYFRAME_SERVER_KEYFRAME_EDITOR_H

#include <rviz/display.h>
#include "ui_keyframe_editor.h"

#include <nimbro_keyframe_server/motion.h>
#include <nimbro_keyframe_server/MotionList.h>
#include <nimbro_keyframe_server/PlayMotionAction.h>

#ifndef Q_MOC_RUN

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <rviz/properties/string_property.h>

#include <actionlib/client/simple_action_client.h>

#include <moveit/planning_scene/planning_scene.h>
#include <QTimer>
#include <QRadioButton>
#include <ros/subscriber.h>

#endif

#include "motion_model.h"
#include "ik_view.h"
#include "joint_space_view.h"

namespace nimbro_keyframe_editor
{

class KeyFrameEditor : public rviz::Display
{
Q_OBJECT
public:
	KeyFrameEditor();
	virtual ~KeyFrameEditor();

	virtual void onInitialize();
	virtual void update(float wall_dt, float ros_dt);
	virtual void onEnable();
Q_SIGNALS:
	void jointStateReceived(const sensor_msgs::JointStateConstPtr& msg);
	void actionClientFinished();
	void actionClientFeedback(const nimbro_keyframe_server::PlayMotionFeedbackConstPtr& feedback, int kf_offset = 0);
private Q_SLOTS:
	void loadMotion(const QString& name);
	void duplicate();
	void remove();
	void save();
	void play();
	void playFrame();
	void revert();
	void setFromBot();
	void testMotion();
	void toggleStream();
	void streamMotion();
	void handleMoveGroupButtons(bool checked);
	void handleChangedSelection(const QItemSelection& selection);

	void updateFromState();
	void updateFromModel();

	void handleJointState(const sensor_msgs::JointStateConstPtr& msg);
	void streamMotionCb();

	void checkActionClient();
	void handleActionFeedback(const nimbro_keyframe_server::PlayMotionFeedbackConstPtr& feedback, int kf_offset);

private:
	void handleMotionList(const nimbro_keyframe_server::MotionList& msg);

	int selectedKeyFrameIndex();

	void setCurrentStateAsMotion();

	std::string m_keyframeServer;

	typedef std::map<std::string, std::map<nimbro_keyframe_server::KeyFrame::InterpolationSpace, QRadioButton*> > BtnMap;

	robot_model_loader::RobotModelLoaderPtr m_loader;

	QWidget* m_w;
	Ui::KeyFrameEditor m_ui;
	ros::Subscriber m_sub_motionList;

	robot_model::RobotModelPtr m_model;
	robot_state::RobotStatePtr m_state;
	planning_scene::PlanningScenePtr m_planningScene;
	robot_state::RobotStatePtr m_currentRobotState;
	const robot_model::JointModelGroup* m_group;
	std::vector<std::string> m_groupNames;

	moveit_rviz_plugin::RobotStateVisualizationPtr m_vis;

	rviz::Display* m_intDisplay;

	MotionModel m_motionModel;
	IKView* m_keyFrameView;
	JointSpaceView* m_jointSpaceView;
	std::vector<QRadioButton*> m_groupSelectButtons;
	BtnMap m_groupButtons;

	typedef actionlib::SimpleActionClient<nimbro_keyframe_server::PlayMotionAction> ActionClient;
	boost::shared_ptr<ActionClient> m_play_motion_action_client;
	bool m_actionActive = false;

	nimbro_keyframe_server::Motion m_streamMotion;

	ros::Subscriber m_sub_js;
	ros::Publisher m_markerPub;
	ros::Publisher m_pub_plot;
	bool m_disableKeyframes;

	std::vector<bool> m_ignore;
	std::map<std::string, std::string> m_jointNameToJointGroupName;
	std::map<std::string, double> m_jointNameToLowerLimit;
	std::map<std::string, double> m_jointNameToUpperLimit;

	void initMappings();

	void testSelfCollision();

	std::map<std::string, std::tuple<double, double, double>> m_linkColors;
	std::map<std::string, bool> m_linkInCollision;

	std::string m_setMotionServiceName;

	QTimer* m_streamMotionTimer;

	void publishReferenceObjectPose(const Eigen::Affine3d& pose, const std::string& path);
	ros::Publisher m_pub_referencePose;
};

}

#endif
