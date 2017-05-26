#include <apc_control/states/compute_motions.h>

namespace kfs = nimbro_keyframe_server;

namespace apc_control
{



void ComputeMotions::compute_pick_pencil_cup(bool standing)
{

// 	Eigen::Affine3d pose_arm_ik25 = m_pose * driver()->getFinger25ArmTransform().inverse();
// 	Eigen::Affine3d pose_arm_ik05 = m_pose * driver()->getFinger05ArmTransform().inverse();
	Eigen::Affine3d transl_ik25;
	transl_ik25 = Eigen::Translation3d(m_pose.translation()) * driver()->getFinger25ArmTransform().inverse();

	const Eigen::Affine3d cupBottom(Eigen::Translation3d(m_pose.translation()));
	const Eigen::Affine3d ik25(driver()->getFinger25ArmTransform().inverse());

//Pregrasp
	{
		kfs::Motion::Ptr pregrasp(new kfs::Motion);
		pregrasp->setName("pregrasp_pencil_cup");
		if(standing)
		{
			kfs::KeyFrame kf;
			kf.setLabel("pregrasp pencil cup standing");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
			kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);

			kf.setJointSpaceVelocity(0.2);
			kf.setCartLinearVelocity(0.1);
			kf.setAngularVelocity(0.2);

			// First kf: above cup
			Eigen::Affine3d aboveCup = cupBottom;
			aboveCup.translation().z() = m_box_height;

			kf.setState("arm", aboveCup * ik25);
			kf.setJointPosition("suc_finger_joint", 0.25);
			kf.setJointPosition("suc_fingertip_joint", 0.0);
			pregrasp->push_back(kf);

			// Turn finger downwards
			kf.setJointPosition("suc_fingertip_joint", 30.0 * M_PI / 180.0);
			pregrasp->push_back(kf);

			{
				Eigen::Affine3d atCupHeight = cupBottom;
				atCupHeight.translation().z() += 0.16;
				atCupHeight.translation().x() -= 0.03;

				kf.setCartLinearVelocity(0.05);
				kf.setState("arm", atCupHeight * ik25);
				pregrasp->push_back(kf);
			}

			{
				Eigen::Affine3d tiltCup = cupBottom;
				tiltCup.translation().z() += 0.135;
				tiltCup.translation().x() -= 0.06;

				kf.setState("arm", tiltCup * ik25);
				pregrasp->push_back(kf);
			}

			// Second kf: at grasp height
			Eigen::Affine3d graspHeightPose = cupBottom;
			graspHeightPose.translation().z() += 0.06;
			graspHeightPose.translation().x() += 0.03;

			kf.setCartLinearVelocity(0.05);
			kf.setState("arm", graspHeightPose * ik25);
			kf.setJointPosition("suc_fingertip_joint", 0.0);
			pregrasp->push_back(kf);
		}
		else
		{
			kfs::KeyFrame kf;
			kf.setLabel("pregrasp pencil cup");
			kf.addJointGroup("arm_with_eef", kfs::KeyFrame::IS_CARTESIAN);

			Eigen::Affine3d pose;
			pose = m_pose * Eigen::Translation3d(-0.05, 0, 0);

			kf.setCartLinearVelocity(0.3);
			kf.setAngularVelocity(0.2);
			kf.setState("arm_with_eef", pose);

			pregrasp->push_back(kf);
		}

		driver()->setPregrasp(pregrasp);
	}

//Grasp
	//1st grasp motion
	{
		if(standing)
		{
			kfs::Motion::Ptr grasp(new kfs::Motion);
			grasp->setName("grasp pencil cup");

			kfs::KeyFrame kf;
			kf.setLabel("grasp pencil cup");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);

			Eigen::Affine3d pose;
			pose = Eigen::Translation3d(0.03,0.0,0.04) * transl_ik25;

			kf.setState("arm", pose);
			kf.setCartLinearVelocity(0.15);
			kf.setAngularVelocity(0.1);
			grasp->push_back(kf);
			driver()->setGrasp(grasp);
		}
		else
		{
			kfs::Motion::Ptr grasp(new kfs::Motion);
			grasp->setName("grasp pencil cup");

			kfs::KeyFrame kf;
			kf.setLabel("grasp pencil cup");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
			kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);

			Eigen::Affine3d pose;
			pose = Eigen::Translation3d(0.05,0,0) * transl_ik25;

			kf.setState("arm", pose);
			kf.setJointPosition("suc_finger_joint" , 0.25);
			kf.setJointPosition("suc_fingertip_joint", 0);
			kf.setCartLinearVelocity(0.15);
			kf.setAngularVelocity(0.1);
			grasp->push_back(kf);
			driver()->setGrasp(grasp);
		}
	}

	//2nd grasp motion (push harder)
	{
		if(standing)
		{
			kfs::Motion::Ptr grasp(new kfs::Motion);
			grasp->setName("grasp2 pencil cup");

			kfs::KeyFrame kf;
			kf.setLabel("grasp2 pencil cup");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
			kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);

			Eigen::Affine3d pose;
			pose = Eigen::Translation3d(0.08,0.0,0.04) * transl_ik25;
			pose.translation().x() = 1.152;

			kf.setState("arm", pose);
			kf.setJointPosition("suc_finger_joint", 0.36);
			kf.setCartLinearVelocity(0.15);
			kf.setJointSpaceVelocity(0.05);
			kf.setAngularVelocity(0.1);

			grasp->push_back(kf);
			driver()->setGrasp2(grasp);
		}
		else
		{
			kfs::Motion::Ptr grasp(new kfs::Motion);
			grasp->setName("grasp pencil cup");

			kfs::KeyFrame kf;
			kf.setLabel("grasp pencil cup");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
			kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);

			Eigen::Affine3d pose;
			pose = Eigen::Translation3d(0.08,0,0) * transl_ik25;
			pose.translation().x() = 1.152;
			kf.setState("arm", pose);
			kf.setJointPosition("suc_finger_joint" , 0.36);
			kf.setJointPosition("suc_fingertip_joint", 0);
			kf.setCartLinearVelocity(0.15);
			kf.setAngularVelocity(0.1);
			grasp->push_back(kf);
			driver()->setGrasp2(grasp);

		}
	}

//Retract motion

	kfs::Motion::Ptr retract(new kfs::Motion);
	retract->setName("retract pencil cup");

	{ //1st frame lift finger and keep clear of the sides of the shelf, they might snag items

		if(standing)
		{
			kfs::KeyFrame kf;
			kf.setLabel("retract lift item pencil cup");
			kf.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);

			Eigen::Affine3d pose = m_pose;
			pose.translation().z() = m_box_height - 0.03;
			pose = Eigen::Translation3d(pose.translation()) * driver()->getFinger25ArmTransform().inverse();

			kf.setState("arm", pose);
			kf.setCartLinearVelocity(0.15);
			kf.setAngularVelocity(0.05);
			retract->push_back(kf);

			//2nd and 3rd keyframe in front of the shelf + a 4th in case of a side grasp
			kfs::KeyFrame kf2;
			kf2.setLabel("retract from shelf pencil cup");
			kf2.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);
			kf2.addJointGroup("arm", kfs::KeyFrame::IS_CARTESIAN);
			kf2.setAngularVelocity(0.2);
			kf2.setCartLinearVelocity(0.2);
			kf2.setJointSpaceVelocity(0.08);

			pose.translation().x() = 0.9f;

			kf2.setJointPosition("suc_finger_joint", 0.05);
			kf2.setState("arm",pose);


			retract->push_back(kf2);
			driver()->setRetract(retract);

		}
#if 0
		// Keep clear of the sides of the shelf, they might snag items
		if(driver()->getCurrentItem().loc_col == 0)
		{
			float dist2Side = driver()->perceptionResult()->box_coordinates.boxCoordinates[3] - pose.translation().y();
			ROS_INFO("left col: dist2Side: %f", dist2Side);
			if(dist2Side < 0.1)
			{
				pose.translation().y() -= 0.03;
				kf.setState("arm", pose);
				retract->push_back(kf);
			}
		}
		else if(driver()->getCurrentItem().loc_col == 2)
		{
			float dist2Side = pose.translation().y() - driver()->perceptionResult()->box_coordinates.boxCoordinates[2];
			ROS_INFO("right col: dist2Side: %f", dist2Side);
			if(dist2Side < 0.1)
			{
				pose.translation().y() += 0.03;
				kf.setState("arm", pose);
				retract->push_back(kf);
			}
		}
#endif




		// If this was a side grasp, add keyframe with cup pointing downwards
// 		pose = Eigen::Translation3d(pose.translation()) * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
// 		kf2.setState("arm", pose);
// 		kf2.setAngularVelocity(0.4);
// 		retract->push_back(kf2);
	}


	{
		kfs::Motion::Ptr tote_release(new kfs::Motion);
		tote_release->setName("tote_release");

		kfs::KeyFrame kf;
		kf.setLabel("tote_retract");
		kf.setCartLinearVelocity(0.3);
		kf.setAngularVelocity(0.4);
		kf.addJointGroup("arm",kfs::KeyFrame::IS_CARTESIAN);

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

		pose = Eigen::Translation3d(0.83, y, 0.9) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * driver()->getFinger05ArmTransform().inverse();

		kf.setState("arm", pose);
		tote_release->push_back(kf);

		kf.addJointGroup("arm_eef", kfs::KeyFrame::IS_JOINT_SPACE);
		kf.setJointPosition("suc_finger_joint", 0.25);

		pose = Eigen::Translation3d(0.77, y, 0.6) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * driver()->getFinger25ArmTransform().inverse();

		kf.setState("arm", pose);
		kf.setAngularVelocity(0.4);
		kf.setCartLinearVelocity(0.6);
		tote_release->push_back(kf);

		driver()->setToteRelease(tote_release);
	}



	}



}
