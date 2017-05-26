#include <apc_control/states/compute_motions.h>

namespace kfs = nimbro_keyframe_server;

namespace apc_control
{

void ComputeMotions::compute_stow_pencil_cup(bool standing)
{
	Eigen::Affine3d link_transform25 = driver()->getFinger25ArmTransform();
	Eigen::Affine3d link_transform05 = driver()->getFinger05ArmTransform();
	Eigen::Affine3d pose_arm_ik = m_pose * link_transform25.inverse();


//Pregrasp

	if(!standing)
	{
		compute_stow_pregrasp();
	}
	else
	{
		kfs::Motion::Ptr pregrasp(new kfs::Motion);
		pregrasp->setName("tote_pregrasp_pencil");

		//Keyframe above item
		{
			kfs::KeyFrame kf;
			kf.setLabel("above pencil cup");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
			kf.setLinearVelocity(0.3);
			kf.setAngularVelocity(1.4);

			Eigen::Affine3d pose;

			pose =	Eigen::Translation3d(m_pose.translation().x(), m_pose.translation().y(), 0.85)
					* Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()) * link_transform25.inverse();

			kf.setState("arm", pose);
			pregrasp->push_back(kf);



			Eigen::Vector3d gdirection = m_pose.rotation() * Eigen::Vector3d::UnitX();
			if(gdirection.x() > std::sin(15 * M_PI / 180))
			{
				float jpos = -0.324407 - M_PI;
				if(gdirection.y() > 0)
					jpos += 2*M_PI;
				kfs::KeyFrame kf2;
				kf2.setLabel("rotate arm");
				kf2.addJointGroup("arm", kfs::KeyFrame::IS_JOINT_SPACE);
				kf2.setLinearVelocity(2.5);
				kf2.setJointPosition("wrist_3_joint", jpos);
				pregrasp->push_back(kf2);
			}


			pose = m_pose * Eigen::Translation3d(-0.15, 0, 0);
			kf.setLabel("pregrasp_pencil_cup_side");
			kf.setState("arm_with_eef", pose);
			pregrasp->push_back(kf);
		}

		//Keyframe stretch finger
		{
			kfs::KeyFrame kf;
			kf.setLabel("stretch finger");
			kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);
			kf.setLinearVelocity(4.0);
			kf.setJointPosition("suc_fingertip_joint", 0.0);
			pregrasp->push_back(kf);
			kf.setJointPosition("suc_finger_joint",0.25);

			pregrasp->push_back(kf);
		}


		driver()->setPregrasp(pregrasp);
	}



//Grasp

	if(standing)
	{

		//1st grasp motion
		{
			kfs::Motion::Ptr grasp(new kfs::Motion);
			grasp->setName("grasp_pencil_cup_top");

			kfs::KeyFrame kf;
			kf.setLabel("grasp grasp_pencil_cup_top cup");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);

			Eigen::Affine3d pose;
			pose = Eigen::Translation3d(0,0,-0.01) * pose_arm_ik;

			kf.setState("arm", pose);
			kf.setLinearVelocity(0.15);
			kf.setAngularVelocity(0.1);
			grasp->push_back(kf);

			driver()->setGrasp(grasp);
		}

		//2nd grasp motion (push harder)
		{
			kfs::Motion::Ptr grasp(new kfs::Motion);
			grasp->setName("grasp2_pencil_cup_top");

			kfs::KeyFrame kf;
			kf.setLabel("grasp2_pencil_cup_top");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);

			Eigen::Affine3d pose;
			pose = Eigen::Translation3d(0,0,-0.03) * pose_arm_ik;

			kf.setState("arm", pose);
			kf.setLinearVelocity(0.15);
			kf.setAngularVelocity(0.1);
			grasp->push_back(kf);

			driver()->setGrasp2(grasp);
		}
	}
	else //not standing
	{
		//Grasp 1
		{
			kfs::Motion::Ptr grasp(new kfs::Motion);
			grasp->setName("grasp_pencil_cup_side");

			kfs::KeyFrame kf;
			kf.setLabel("grasp_pencil_cup_side");
			kf.addJointGroup("arm_with_eef", kfs::KeyFrame::IS_CARTESIAN);

			Eigen::Affine3d pose;
			pose = m_pose * Eigen::Translation3d(0.01,0.0,0.0);

			kf.setState(m_graspGroup, pose);
			kf.setLinearVelocity(0.15);
			kf.setAngularVelocity(0.1);
			grasp->push_back(kf);

			driver()->setGrasp(grasp);
		}

		//Grasp2
		{
			kfs::MotionPtr grasp2(new kfs::Motion);
			grasp2->setName("grasp2_pencil_cup_side");
			kfs::KeyFrame kf;
			kf.setLabel("grasp2_pencil_cup_top");
			kf.addJointGroup("arm_with_eef",kfs::KeyFrame::IS_CARTESIAN);
			kf.setLinearVelocity(0.15);
			kf.setAngularVelocity(0.1);

			//move inside the cup
			Eigen::Affine3d pose;
			pose = m_pose * Eigen::Translation3d(0.05,0,0);
			kf.setState("arm_with_eef", pose);
			grasp2->push_back(kf);

			

			//flip it up
			pose =  Eigen::Translation3d(0, 0, 0.09) * pose *  Eigen::Translation3d(0.15, 0, 0);
			kf.setState("arm_with_eef", pose);
			grasp2->push_back(kf);

			//move little further in x-direction
			pose = pose * Eigen::Translation3d(0.05, 0,0);
			kf.setState("arm_with_eef", pose);
			grasp2->push_back(kf);


			//stretch finger
			kfs::KeyFrame kf2;
			kf2.setLabel("grasp2_pencil_cup_side");
			kf2.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
			kf2.setAngularVelocity(0.1);
			kf2.setLinearVelocity(0.1);

			//switch kinematic
			Eigen::Affine3d pose_arm = Eigen::Translation3d(pose.translation()) * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()) * link_transform25.inverse();
			kf2.setState("arm", pose_arm);
			kf2.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);
			kf2.setJointPosition("suc_finger_joint", 0.25);
			kf2.setJointPosition("suc_fingertip_joint", 0);
			grasp2->push_back(kf2);

			pose_arm = pose_arm * Eigen::Translation3d(0.11, 0, 0);
			kf2.setState("arm", pose_arm);
			grasp2->push_back(kf2);

			driver()->setGrasp2(grasp2);

		}

	}
//Retract

	kfs::Motion::Ptr retract(new kfs::Motion);
	retract->setName("retract_tote_pencil_cup");
	{ 	//Lift arm
		kfs::KeyFrame kf;

		kf.setLinearVelocity(0.2);
		kf.setLabel("retract tote pencil cup");
		kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);
		kf.setJointPosition("suc_fingertip_joint", 0);
		kf.setJointPosition("suc_finger_joint",0.05);
		retract->push_back(kf);

	}

	Eigen::Affine3d liftPose;

	liftPose = Eigen::Translation3d(0.7, 0.0, 0.85) * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()) * link_transform05.inverse();
	{ //lift arm andf turn suction cup downwards
		kfs::KeyFrame kf;
		kf.setAngularVelocity(0.3);
		kf.setLinearVelocity(0.1);
		kf.setLabel("retract tote");
		kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
		kf.setState("arm", liftPose);
		retract->push_back(kf);

	}
	{	//Rotate Tube to init position.
		kfs::KeyFrame kf;
		kf.setLabel("rotate tube");
		kf.addJointGroup("arm", kfs::KeyFrame::IS_JOINT_SPACE);
		kf.setJointPosition("wrist_3_joint", -0.324407);
		kf.setLinearVelocity(0.4);
		retract->push_back(kf);
	}

	driver()->setRetract(retract);


//StowBox

	std::vector<float> box_coordinates = apc_shelf_model::getBoxCoordinates(m_current_item.dest_row,m_current_item.dest_col);

	{
		kfs::Motion::Ptr motion(new kfs::Motion);
		motion->setName("stow_box_pencil_cup");
		kfs::KeyFrame kf;

		Eigen::Affine3d pose = Eigen::Translation3d(
		1.1,
		0.5 * box_coordinates[2] + 0.5 * box_coordinates[3],
		m_box_height)
		* link_transform05.inverse();

		kf.setAngularVelocity(0.3);
		kf.setLinearVelocity(0.3);
		kf.setLabel("[shelf] stow_box pencil cup");
		kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
		kf.setState("arm", pose);
		motion->push_back(kf);

		pose = Eigen::Translation3d(0.15,0,0) * pose;
		kf.setState("arm", pose);
		motion->push_back(kf);

		driver()->setStowBox(motion);
	}

//StowPlace

	{

		kfs::Motion::Ptr motion(new kfs::Motion);
		motion->setName("stow_item_pencil_cup");

		// Push arm into the box
		nimbro_keyframe_server::KeyFrame kf;

		{
			kf.setLabel("[shelf] push pencil cup into box");
			kf.setLinearVelocity(0.2);
			kf.addJointGroup("arm_eef", nimbro_keyframe_server::KeyFrame::IS_JOINT_SPACE);
			kf.setJointPosition("suc_finger_joint", 0.3);
			kf.setJointPosition("suc_fingertip_joint", 30 * M_PI / 180);

			motion->push_back(kf);
		}
		driver()->setStowPlace(motion);
	}


}


}//namespace
