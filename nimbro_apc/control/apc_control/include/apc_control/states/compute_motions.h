// Motion Computation state for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_CONTROL_COMPUTE_MOTIONS_H
#define APC_CONTROL_COMPUTE_MOTIONS_H

#include <geometry_msgs/PoseStamped.h>

#include <apc_control/states/state.h>
#include <nimbro_fsm/fsm_ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <apc_control/apc_database.h>
#include <apc_shelf_model/dimensions.h>


namespace apc_control
{

	class ComputeMotions : public State
	{
	public:
		virtual void enter() override;
		virtual nimbro_fsm::StateBase* execute() override;
		virtual void exit() override;
	private:
		Eigen::Affine3d m_pose;
		float m_box_height;
		WorkItem m_current_item;
		std::string m_graspGroup;

		void compute_pick_motions();
		void compute_stow_motions();

		void compute_pick_pregrasp();
		void compute_pick_grasp();
		void compute_pick_retract();

		void compute_pick_tote_release();

		void compute_stow_pregrasp();
		void compute_stow_grasp();
		void compute_stow_retract();
		void compute_stow_box();

		void compute_stow_place();

		void compute_pick_pencil_cup(bool standing);
		void compute_stow_pencil_cup(bool standing);

	};

}

#endif
