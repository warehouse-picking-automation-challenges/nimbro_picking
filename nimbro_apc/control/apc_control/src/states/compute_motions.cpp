// Perception state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/console.h>
#include <apc_control/control.h>
#include <apc_control/states/compute_motions.h>
#include <apc_control/states/picking/box_pose.h>
#include <apc_control/states/stowing/tote_pregrasp.h>
#include <nimbro_keyframe_server/motion.h>

#include <eigen3/Eigen/src/Geometry/Transform.h>

namespace kfs = nimbro_keyframe_server;

namespace apc_control
{

void ComputeMotions::compute_pick_pregrasp()
{
	kfs::Motion::Ptr pregrasp(new kfs::Motion);
	pregrasp->setName("pregrasp");

	kfs::KeyFrame kf;
	kf.setLabel("pregrasp");
	kf.addJointGroup("arm_with_eef", kfs::KeyFrame::IS_CARTESIAN);

	Eigen::Affine3d pose;
	pose = m_pose * Eigen::Translation3d(-0.05,0.0,0.0);

	ROS_INFO_STREAM("pick pregrasp grasp pose:\n" << pose.matrix());

	float box_Z_space = m_box_height - pose.translation().z();

	if(box_Z_space <= 0.0)
	{
		pose.translation().z() = m_box_height;
		ROS_WARN("not enough space (%f cm) above the finger! Changed height Z value to %f",box_Z_space, m_box_height);
	}

	kf.setState("arm_with_eef", pose);
	kf.setCartLinearVelocity(0.14);
	kf.setAngularVelocity(0.2);
	pregrasp->push_back(kf);

	driver()->setPregrasp(pregrasp);
}

void ComputeMotions::compute_pick_grasp()
{
	//1st grasp motion
	{
		kfs::Motion::Ptr grasp(new kfs::Motion);
		grasp->setName("grasp");

		kfs::KeyFrame kf;
		kf.setLabel("grasp");
		kf.addJointGroup(m_graspGroup, kfs::KeyFrame::IS_CARTESIAN);

		Eigen::Affine3d pose;
		pose = m_pose * Eigen::Translation3d(0.01,0.0,0.0);

		ROS_INFO_STREAM("pick grasp grasp pose:\n" << pose.matrix());

		kf.setState(m_graspGroup, pose);
		kf.setCartLinearVelocity(0.15);
		kf.setAngularVelocity(0.1);
		grasp->push_back(kf);

		driver()->setGrasp(grasp);
	}

	//2nd grasp motion (push harder)
	{
		kfs::Motion::Ptr grasp(new kfs::Motion);
		grasp->setName("grasp2");

		kfs::KeyFrame kf;
		kf.setLabel("grasp2");
		kf.addJointGroup(m_graspGroup, kfs::KeyFrame::IS_CARTESIAN);

		Eigen::Affine3d pose;
		pose = m_pose * Eigen::Translation3d(0.03,0.0,0.0);

		kf.setState(m_graspGroup, pose);
		kf.setCartLinearVelocity(0.15);
		kf.setAngularVelocity(0.1);
		grasp->push_back(kf);

		driver()->setGrasp2(grasp);
	}
}

void ComputeMotions::compute_pick_retract()
{
	//Retract motion
	Eigen::Affine3d pose = m_pose;
	kfs::Motion::Ptr retract(new kfs::Motion);
	retract->setName("retract");

	{ //1st frame lift finger and keep clear of the sides of the shelf, they might snag items
		kfs::KeyFrame kf;
		kf.setLabel("retract lift item");
		kf.addJointGroup("arm_with_eef", kfs::KeyFrame::IS_CARTESIAN);

		float box_Z_space = m_box_height - m_pose.translation().z();
		if(box_Z_space < 0.01)
				ROS_WARN("not enough space (%f cm) above the finger!",box_Z_space);
		else
		{
			pose.translation().z() = m_box_height;
			// HACK: Ignore orientation, replace with 45° pitch
			if(pose.rotation().col(0).x() > 0.5)
			{
				ROS_WARN("HACK: 45° pitch for top grasp!");
				pose = Eigen::Translation3d(pose.translation()) * Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitY());
			}
		}
		ROS_INFO_STREAM("pick retract grasp pose:\n" << pose.matrix());

		kf.setState("arm_with_eef", pose);
		kf.setCartLinearVelocity(0.2);
		kf.setAngularVelocity(0.2);
		retract->push_back(kf);





		//HACK for paper kleenex_paper_towels
		if(m_current_item.name == "kleenex_paper_towels")
		{
			if(driver()->getCurrentItem().loc_col == 0)
			{
				pose.translation().y() -=0.03;
			}
			else if(driver()->getCurrentItem().loc_col == 2)
			{
				pose.translation().y() += 0.03;
			}
		}
		else
		{
			pose.translation().y() = driver()->perceptionResult()->retractPositionY;
		}
		kf.setState("arm_with_eef", pose);
		retract->push_back(kf);
	}

	{ //2nd and 3rd keyframe in front of the shelf + a 4th in case of a side grasp
		kfs::KeyFrame kf;
		kf.setLabel("retract from shelf");
		kf.addJointGroup("arm_with_eef", kfs::KeyFrame::IS_CARTESIAN);
		kf.setAngularVelocity(0.3);
		kf.setCartLinearVelocity(0.1);

		pose.translation().x() = 1.08f;
		kf.setState("arm_with_eef", pose);
		retract->push_back(kf);

		pose.translation().z() += 0.03;
		kf.setState("arm_with_eef", pose);
		retract->push_back(kf);

		Eigen::Vector3d cupDirection = pose.rotation() * Eigen::Vector3d::UnitX();

		// If this was a side grasp, add keyframe with cup pointing downwards
		if(std::abs(cupDirection.y()) > std::sin(M_PI/4.0))
		{
			pose = Eigen::Translation3d(pose.translation()) * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
			kf.setState("arm_with_eef", pose);
			kf.setAngularVelocity(0.4);
			retract->push_back(kf);
		}
	}

	driver()->setRetract(retract);

}

void ComputeMotions::compute_pick_tote_release()
{
	kfs::Motion::Ptr tote_release(new kfs::Motion);
	tote_release->setName("tote_release");

	kfs::KeyFrame kf;
	kf.setLabel("tote_retract");
	kf.setCartLinearVelocity(0.3);
	kf.setAngularVelocity(0.4);
	kf.addJointGroup("arm_with_eef",kfs::KeyFrame::IS_CARTESIAN);

	double y=0;
	switch(m_current_item.loc_col)
	{
		case 0:
			y = 0.12;
			break;
		case 2:
			y = -0.12;
			break;
		case 1:
		default:
			y=0;
			break;
	}
	Eigen::Affine3d pose;

	pose = Eigen::Translation3d(0.83, y, 0.9) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());

	kf.setState("arm_with_eef", pose);
	tote_release->push_back(kf);

	pose = Eigen::Translation3d(0.77, y, 0.6) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
	kf.setState("arm_with_eef", pose);
	kf.setAngularVelocity(0.4);
	kf.setCartLinearVelocity(0.6);
	tote_release->push_back(kf);

	driver()->setToteRelease(tote_release);
}


void ComputeMotions::compute_stow_pregrasp()
{
	kfs::Motion::Ptr pregrasp(new kfs::Motion);
	pregrasp->setName("tote_pregrasp");

	//Keyframe above item
	kfs::KeyFrame kf;
	kf.setLabel("pose above item");
	kf.addJointGroup(m_graspGroup, kfs::KeyFrame::IS_CARTESIAN);

	Eigen::Affine3d pose;

	pose =	Eigen::Translation3d(m_pose.translation().x(), m_pose.translation().y(), 0.85)
			* Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());

	ROS_INFO_STREAM("stow pregrasp grasp pose:\n" << pose.matrix());

	kf.setState(m_graspGroup, pose);
	kf.setCartLinearVelocity(0.3);
	kf.setAngularVelocity(1.4);
	pregrasp->push_back(kf);

	//rotate arm if grasp pose is pointing away from robot
	Eigen::Vector3d gdirection = m_pose.rotation() * Eigen::Vector3d::UnitX();
	if(gdirection.x() > std::sin(15 * M_PI / 180))
	{
		float jpos = -0.324407 - M_PI;
		if(gdirection.y() > 0)
			jpos += 2*M_PI;
		kfs::KeyFrame kf2;
		kf2.setLabel("rotate arm");
		kf2.addJointGroup("arm", kfs::KeyFrame::IS_JOINT_SPACE);
		kf2.setJointSpaceVelocity(2.5);
		kf2.setJointPosition("wrist_3_joint", jpos);
		pregrasp->push_back(kf2);
	}

	//2nd keyframe pregrasp

	pose = m_pose * Eigen::Translation3d(-0.05,0.0, 0.0);
	kf.setState(m_graspGroup, pose);
	kf.setCartLinearVelocity(0.1);
	pregrasp->push_back(kf);

	driver()->setPregrasp(pregrasp);

}

void ComputeMotions::compute_stow_grasp()
{
	compute_pick_grasp();
}

void ComputeMotions::compute_stow_retract()
{
	Eigen::Affine3d pose = m_pose;

	ROS_INFO_STREAM("stow retract grasp pose:\n" << pose.matrix());

	pose.translation().z() += 0.25;
	kfs::Motion::Ptr retract(new kfs::Motion);
	retract->setName("retract_tote");
	{ 	//Lift arm
		kfs::KeyFrame kf;

		kf.setAngularVelocity(0.4);
		kf.setCartLinearVelocity(0.4);
		kf.setLabel("retract_tote");
		kf.addJointGroup(m_graspGroup, kfs::KeyFrame::IS_CARTESIAN);
		kf.setState(m_graspGroup, pose);
		retract->push_back(kf);

	}

	Eigen::Vector3d gdirection = m_pose.rotation() * Eigen::Vector3d::UnitX();

	if(gdirection.x() > std::sin(15 * M_PI / 180))
	{	//Rotate Tube to init position.
		kfs::KeyFrame kf;
		kf.setLabel("rotate tube");
		kf.addJointGroup("arm", kfs::KeyFrame::IS_JOINT_SPACE);
		kf.setJointPosition("wrist_3_joint", -0.324407);
		kf.setJointSpaceVelocity(2.5);
		retract->push_back(kf);
	}

	pose.translation().x() = 0.7;
	pose.translation().y() = 0.0;
	pose.translation().z() = 0.85;
	Eigen::Affine3d liftPose;
	liftPose = Eigen::Translation3d(pose.translation()) * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
	{ //lift arm andf turn suction cup downwards
		kfs::KeyFrame kf;
		kf.setAngularVelocity(0.4);
		kf.setCartLinearVelocity(0.3);
		kf.setLabel("retract_tote");
		kf.addJointGroup("arm_with_eef", kfs::KeyFrame::IS_CARTESIAN);
		kf.setState("arm_with_eef", liftPose);
		retract->push_back(kf);

	}
	{	//Rotate Tube to init position.
		kfs::KeyFrame kf;
		kf.setLabel("rotate tube");
		kf.addJointGroup("arm", kfs::KeyFrame::IS_JOINT_SPACE);
		kf.setJointPosition("wrist_3_joint", -0.324407);
		kf.setJointSpaceVelocity(0.4);
		retract->push_back(kf);
	}

	driver()->setRetract(retract);
}

void ComputeMotions::compute_stow_box()
{

	std::vector<float> box_coordiantes = apc_shelf_model::getBoxCoordinates(m_current_item.dest_row,m_current_item.dest_col);

	float angle = (M_PI/2.0) - (20 * M_PI / 180);

	Eigen::Affine3d pose = Eigen::Translation3d(
		1.1,
		0.5 * box_coordiantes[2] + 0.5 * box_coordiantes[3],
		m_box_height)
		* Eigen::AngleAxisd(angle , Eigen::Vector3d::UnitY());


	kfs::Motion::Ptr motion(new kfs::Motion);
	motion->setName("stow_box");
	kfs::KeyFrame kf;

	kf.setAngularVelocity(0.3);
	kf.setCartLinearVelocity(0.3);
	kf.setLabel("[shelf] stow_box");
	kf.addJointGroup("arm_with_eef", kfs::KeyFrame::IS_CARTESIAN);
	kf.setState("arm_with_eef", pose);
	motion->push_back(kf);

	pose.translation().x() = 1.3;
	kf.setState("arm_with_eef", pose);
	motion->push_back(kf);

	driver()->setStowBox(motion);
}

void ComputeMotions::compute_stow_place()
{
	std::vector<float> box_coordinates = apc_shelf_model::getBoxCoordinates(m_current_item.dest_row,m_current_item.dest_col);

	float angle = (M_PI/2.0) - (20 * M_PI / 180);

	Eigen::Affine3d boxPose;
	boxPose = Eigen::Translation3d(
		1.55,
		0.5 * box_coordinates[2] + 0.5 * box_coordinates[3], //mean min and max Y
		m_box_height
	)
		* Eigen::AngleAxisd(angle , Eigen::Vector3d::UnitY());

	kfs::Motion::Ptr motion(new kfs::Motion);
	motion->setName("stow_item");

	// Push arm into the box
	nimbro_keyframe_server::KeyFrame kf;

	{
		kf.setLabel("[shelf] push into box");
		kf.setCartLinearVelocity(0.1);
		kf.setAngularVelocity(0.1);
		kf.addJointGroup("arm_with_eef", nimbro_keyframe_server::KeyFrame::IS_CARTESIAN);
		kf.setState("arm_with_eef", boxPose);

		motion->push_back(kf);
	}
	driver()->setStowPlace(motion);
}


void ComputeMotions::compute_pick_motions()
{
	m_box_height = driver()->getBoxHeight(m_current_item.loc_row);
// 	//Pencil Cup
	if(m_current_item.name == "rolodex_jumbo_pencil_cup")
	{
		compute_pick_pencil_cup(driver()->perceptionResult()->object_standing);
		return;
	}

	//default motions
	compute_pick_pregrasp();
	compute_pick_grasp();
	compute_pick_retract();
	compute_pick_tote_release();
}

template<class T>
T robustAcos(T x)
{
	return std::acos(std::max<T>(-1.0, std::min<T>(1.0, x)));
}

void ComputeMotions::compute_stow_motions()
{
	Eigen::Vector3d xAxis = m_pose.rotation() * Eigen::Vector3d::UnitX();
	Eigen::Vector3d topGraspDir = -Eigen::Vector3d::UnitZ();

	float angleToTopGrasp = robustAcos(xAxis.dot(topGraspDir));

	ROS_INFO_STREAM("xAxis: " << xAxis.transpose());
	ROS_INFO("angle to top grasp: %f", angleToTopGrasp);

	bool topGrasp = std::abs(angleToTopGrasp) < 10.0 * M_PI / 180.0;

	if(topGrasp && m_current_item.objectData->stowGraspOriented)
	{
		ROS_INFO("Oriented grasp!");
		m_graspGroup = "arm_with_eef_orient";
	}
	else
	{
		ROS_INFO("Unoriented grasp!");
		m_graspGroup = "arm_with_eef";
	}

	m_box_height = driver()->getBoxHeight(m_current_item.dest_row);
	if(m_current_item.name == "rolodex_jumbo_pencil_cup")
	{
		compute_stow_pencil_cup(driver()->perceptionResult()->object_standing);
		return;
	}

	//default motions
	compute_stow_pregrasp();
	compute_stow_grasp();
	compute_stow_retract();
	compute_stow_box();
	compute_stow_place();


	std::string path = "/tmp/motion/grasp2_pencil_cup_side.yaml";
	std::ofstream out(path.c_str());

	out << driver()->getGrasp2()->serializeToYAML().c_str();

	ROS_INFO("Stow box motion:\n%s", driver()->getStowBox()->serializeToYAML().c_str());
	ROS_INFO("Stow item motion:\n%s", driver()->getStowPlace()->serializeToYAML().c_str());
}


void ComputeMotions::enter()
{
	State::enter();

	m_graspGroup = "arm_with_eef";

	geometry_msgs::Pose grasp_pose = driver()->getGraspPose().pose;
	tf::poseMsgToEigen(grasp_pose, m_pose);

	m_current_item = driver()->getCurrentItem();

	ROS_INFO_STREAM("grasp pose:\n" << grasp_pose);

	if(driver()->getIsPicking())
		compute_pick_motions();
	else
		compute_stow_motions();

	driver()->setSuctionStrength(1.0f);

}

nimbro_fsm::StateBase* ComputeMotions::execute()
{
	if(driver()->getIsPicking())
		return new apc_control::BoxPose();
	else
		return new apc_control::TotePregrasp();
}

void ComputeMotions::exit()
{
	State::exit();
}

}
