// Perception state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/perception.h>
#include <apc_control/states/compute_motions.h>

#include <apc_control/apc_database.h>
#include <eigen_conversions/eigen_msg.h>

namespace apc_control
{

void PerceptionDelay::enter()
{
	State::enter();

	if(driver()->getIsPicking())
		driver()->switchLight(255);
	else
		driver()->switchLight(128);
	driver()->switchVacuumPower(true);
}

nimbro_fsm::StateBase* PerceptionDelay::execute()
{
	// Wait for any motion to settle down
	if(elapsedTime() > ros::Duration(1.0))
		return new Perception();

	return this;
}

////////////////////////////////////////////////////////////////////////////////

Perception::Perception(bool allItems)
 : m_actionClient("/perception/perception")
 , m_allItems(allItems)
{
}

Perception::~Perception()
{
	if(m_actionClient.isServerConnected())
	{
		if(!m_actionClient.getState().isDone())
			m_actionClient.cancelGoal();
	}
}

void Perception::enter()
{
	State::enter();
	driver()->enableServoCommunication(false);

	if(!driver()->getSimulation())
	{
		ROS_INFO("Enter Perception State");
		if(!m_actionClient.waitForServer())
		{
			ROS_ERROR("Could not wait for action server");
			throw std::runtime_error("Could not wait for action server");
		}

		auto currentItem = driver()->getCurrentItem();

		apc_perception::ApcPerceptionGoal goal;
		goal.box_col = currentItem.loc_col;
		goal.box_row = currentItem.loc_row;

		if(driver()->getIsPicking())
		{
			for(auto& item : driver()->shelf().boxes[currentItem.loc_row][currentItem.loc_col].items)
				goal.candidate_objects.push_back(item.name);

			goal.desired_object = currentItem.name;
		}
		else
		{
			if(m_allItems || driver()->shelf().tote.empty())
			{
				ROS_INFO("Looking for all objects...");
				goal.unsure_candidates = true;
				goal.candidate_objects = driver()->allStowObjects();
			}
			else
			{
				for(auto& item : driver()->shelf().tote)
					goal.candidate_objects.push_back(item.name);
			}

			goal.unwanted_objects = driver()->lastFailedObjects();
		}

		goal.isShelf = driver()->getIsPicking();

		m_actionClient.sendGoal(goal);
	}
	else
	{

		geometry_msgs::PoseStamped grasp_pose;
		if(driver()->getIsPicking()){

			//Box A
			grasp_pose.pose.position.x =  1.4619;
			grasp_pose.pose.position.y =  0.33419;
			grasp_pose.pose.position.z =  1.5358;
			grasp_pose.pose.orientation.x = -0.7069;
			grasp_pose.pose.orientation.y = 0.016176;
			grasp_pose.pose.orientation.z = 0.70692;
			grasp_pose.pose.orientation.w = 0.016176;
		}
		else
		{
		//tote

			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(2.6486, Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(1.107, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(2.7972, Eigen::Vector3d::UnitX());

			Eigen::Quaterniond q (rot);

			grasp_pose.pose.position.x =  0.76048350649302;
			grasp_pose.pose.position.y =  0.11656392336559;
			grasp_pose.pose.position.z =  0.42796093518062;
			grasp_pose.pose.orientation.x = q.x();
			grasp_pose.pose.orientation.y = q.y();
			grasp_pose.pose.orientation.z = q.z();
			grasp_pose.pose.orientation.w = q.w();
		}
		driver()->setGraspPose(grasp_pose);
	}
}

nimbro_fsm::StateBase* Perception::execute()
{

	bool isPicking = driver()->getIsPicking();


	if(driver()->getSimulation())
	{

		if(!isPicking)
			if(!driver()->setWorkItem("up_glucose_bottle"))
				return 0;
		return new ComputeMotions();
	}

	if(elapsedTime() > ros::Duration(40.0))
	{
		//Timeout
		ROS_WARN("Perception TIME OUT after %f sec.", elapsedTime().toSec());

		// HACK: timeout means that the perception detected the item, but could not find a grasp pose.
		driver()->setItemAttempted(true);

		if(!isPicking)
		{
			if(driver()->shelf().workOrder.size() <3)
			{
				ROS_INFO("retry perception with all object labels.");
				return new Perception(true);
			}
		}

		return 0;
	}

	WorkItem current_item;
	if(isPicking)
		current_item=driver()->getCurrentItem();

	ROS_INFO_THROTTLE(2.0, "executing perception State for Item: %s in Box: %s. %f sec elapsed", isPicking ? current_item.name.c_str(): "??" , isPicking ? current_item.location.c_str() : "tote", elapsedTime().toSec());

	switch(m_actionClient.getState().state_)
	{
		case actionlib::SimpleClientGoalState::SUCCEEDED:
			ROS_INFO("Success. Got the following grasps:");

			{
				auto result = m_actionClient.getResult();

				if(result->grasp_poses.empty())
				{
					ROS_ERROR("There was no grasp pose!");

					//retry with all object labels if only 2 items are remaining

					if(!isPicking)
					{
						if(driver()->shelf().workOrder.size() <3)
						{
							ROS_INFO("less than 3 items. retry perception with all object labels.");
							return new Perception(true);
						}
					}

					//next item for picking
					return 0;
				}

				auto poseMsg = result->grasp_poses[0]; //TODO Choose grasp pose
				geometry_msgs::PoseStamped poseInWorld;
				try
				{
					driver()->tf().transformPose("world", poseMsg, poseInWorld);
				}
				catch(tf::TransformException& e)
				{
					ROS_ERROR("Could not transform grasp pose to world frame: %s", e.what());
					return 0;
				}

				driver()->setGraspPose(poseInWorld);
				driver()->setPerceptionResult(result);
				driver()->setItemAttempted(true);

				for(auto& pose : result->grasp_poses)
				{
					ROS_INFO(" - position: %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
				}

				if(!isPicking)
				{
					ROS_INFO("Perception found item '%s'.",result->grasp_object.c_str());
					if(!driver()->setWorkItem(result->grasp_object))
						return 0;


// 					//Rotate arm ?
// 					Eigen::Affine3d gpose;
// 					tf::poseMsgToEigen(poseInWorld.pose, gpose);
//
// 					Eigen::Vector3d gdirection = gpose.rotation() * Eigen::Vector3d::UnitX();
//
// 					if(gdirection.x() > std::sin(15 * M_PI / 180))
// 					{
// 						driver()->setArmRotated(true);
// 						return new apc_control::RotateArm(true, new apc_control::TotePregrasp());
// 					}
//
// 					else
// 						return new apc_control::TotePregrasp();
				}
				return new ComputeMotions();

			}

		case actionlib::SimpleClientGoalState::ABORTED:
			ROS_ERROR("Perception was ABORTED: %s", m_actionClient.getState().getText().c_str());
			return 0;
		case actionlib::SimpleClientGoalState::PENDING:
		case actionlib::SimpleClientGoalState::ACTIVE:
			break;
		case actionlib::SimpleClientGoalState::RECALLED:
		case actionlib::SimpleClientGoalState::REJECTED:
		case actionlib::SimpleClientGoalState::PREEMPTED:
		case actionlib::SimpleClientGoalState::LOST:
		default:
			ROS_ERROR("Unexpected action state '%s', aborting", m_actionClient.getState().getText().c_str());
			return 0;
	}

	return this;
}

void Perception::exit()
{
	driver()->enableServoCommunication(true);
	State::exit();
}

}
