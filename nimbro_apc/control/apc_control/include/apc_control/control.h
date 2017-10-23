// Control node for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_CONTROL_H
#define APC_CONTROL_H

#include <ros/init.h>
#include <nimbro_keyframe_server/motion.h>
#include <nimbro_keyframe_server/PlayMotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include "apc_database.h"
#include <boost/graph/graph_concepts.hpp>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

#include <nimbro_fsm/fsm_ros.h>

#include <apc_interface/ControllerState.h>

#include <apc_perception/ApcPerceptionResult.h>

#include <apc_control/WorkingItem.h>
#include <apc_control/ShelfState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace apc_control
{

class State;

class Control : public nimbro_fsm::Driver
{
public:
	Control();

	void work();
	void init();
	bool m_started = false;

	inline WorkItem getCurrentItem()
	{ return m_currentItem; }

	std::string printWorkItem(WorkItem item);

	inline const Shelf& shelf() const
	{ return m_shelf; }

	std::string report_error(int error_code);

	inline geometry_msgs::PoseStamped getGraspPose()
	{ return m_graspPose; }

	inline void setGraspPose(geometry_msgs::PoseStamped pose)
	{ m_graspPose=pose; }

	inline tf::TransformListener& tf()
	{ return m_tf; }

	bool somethingOnTip() const;


	inline bool objectWellAttached() const
	{ return m_controllerState && m_controllerState->object_well_attached; }

	inline void setItemPlaced(bool success)
	{ m_placed_item=success; }

	inline void setItemGrasped(bool success)
	{ m_grasped_item=success; }

	inline void setItemAttempted(bool attempted)
	{ m_attemped_item = attempted; }

	inline bool getIsPicking()
	{ return m_picking; }

	inline bool getSimulation()
	{ return m_simulation; }

	inline bool getArmRotated()
	{ return m_arm_rotated; }

	inline void setArmRotated(bool r)
	{ m_arm_rotated = r; }

	inline Eigen::Affine3d getFinger25ArmTransform()
	{ return m_finger25arm_transform; }

	inline Eigen::Affine3d getFinger05ArmTransform()
	{ return m_finger05arm_transform; }


	
	bool setWorkItem(std::string itemName);

	float getBoxHeight(int row);

	std::vector<std::string> getBoxItems(int row, int col);

	void switchVacuum(bool mode);
	void switchVacuumPower(bool mode);
	void setSuctionStrength(float value);
	void switchLight(int duty);
	void enableServoCommunication(bool on);


	bool skipThisItem();
	bool handleStart();

	void shutdown();

	Eigen::Affine3d fingerPose() const;

	inline void setPerceptionResult(const apc_perception::ApcPerceptionResultConstPtr& result)
	{ m_perceptionResult = result; }

	inline apc_perception::ApcPerceptionResultConstPtr perceptionResult() const
	{ return m_perceptionResult; }

	void addStateDuration(const std::string& state, const ros::Duration& duration);

	void pushFailedObject(const std::string& obj);
	std::vector<std::string> lastFailedObjects() const;

	std::vector<std::string> allStowObjects();

	void setPayload(float payload);


	//Motions
	typedef nimbro_keyframe_server::Motion::Ptr MotionPtr;

	inline MotionPtr getPregrasp() { return m_pregrasp; }
	inline void setPregrasp(MotionPtr g) {m_pregrasp = g;}

	inline MotionPtr getGrasp() {return m_grasp;}
	inline void setGrasp(MotionPtr g){m_grasp = g;}

	inline MotionPtr getGrasp2() {return m_grasp2;}
	inline void setGrasp2(MotionPtr g){m_grasp2 = g;}

	inline MotionPtr getRetract() {return m_retract; }
	inline void setRetract(MotionPtr g){m_retract = g;}

	inline MotionPtr getStowBox() {return m_stow_box; }
	inline void setStowBox(MotionPtr g){m_stow_box = g;}

	inline MotionPtr getStowPlace() {return m_stow_place; }
	inline void setStowPlace(MotionPtr g){m_stow_place = g;}

	inline MotionPtr getToteRelease() {return m_tote_release; }
	inline void setToteRelease(MotionPtr g){m_tote_release = g;}


private:
	void handleControllerState(const apc_interface::ControllerStateConstPtr& msg);
	void resetItemState();

	ros::NodeHandle m_nh;


	WorkItem m_currentItem;

	using PlayResult = nimbro_keyframe_server::PlayMotionResult;

	std::string m_jsonInputPath;
	std::string m_jsonOutputPath;
	bool m_picking;
	bool m_simulation;
	nimbro_fsm::StateMachineROS m_fsm;

	geometry_msgs::PoseStamped m_graspPose;

	Shelf m_shelf;
	std::vector<WorkItem> m_workOrderCopy;

	tf::TransformListener m_tf;

	bool m_shutdown = false;

	bool m_placed_item;
	bool m_grasped_item;
	bool m_attemped_item;

	bool m_arm_rotated = false;

	ros::Subscriber m_sub_controllerState;
	apc_interface::ControllerStateConstPtr m_controllerState;

	ros::ServiceServer m_srv_skip;
	ros::ServiceServer m_srv_start;

	apc_perception::ApcPerceptionResultConstPtr m_perceptionResult;

	std::map<std::string, ros::Duration> m_stateDurations;

	int bestScore();

	ros::Publisher m_pub_shelfState;
	ShelfState m_shelf_state;

	void updateShelfState();
	void ShelfStateNextItem(int idx);
	void setItemFailed();
	void retryItem();
	int ShelfStateItemIdx(std::string name);

	void WorkOrderStow();
	void WorkOrderPick();
	int m_stow_difficulty;

	void deleteCurrentItemFromWorkOrder();


	//grasp motions
	MotionPtr m_pregrasp;
	MotionPtr m_grasp;
	MotionPtr m_grasp2;
	MotionPtr m_retract;

	//stow motions
	MotionPtr m_stow_box;
	MotionPtr m_stow_place;

	//Pick motions
	MotionPtr m_tote_release;

	robot_model_loader::RobotModelLoaderPtr m_robot_model_loader;
	robot_model::RobotModelPtr m_robot_model;
	robot_state::RobotStatePtr m_robot_state;

	Eigen::Affine3d m_finger25arm_transform;
	Eigen::Affine3d m_finger05arm_transform;
	void computeFingerArmTransform();

	std::vector<std::string> m_lastFailedObjects;

};
}

#endif
