// Kinematics plugin for MoveIt! implementing null-space optimization
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MOMARO_KINEMATICS_H
#define MOMARO_KINEMATICS_H

#include <moveit/kinematics_base/kinematics_base.h>

#include <rbdl/rbdl_parser.h>

#include <config_server/parameter.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <sensor_msgs/JointState.h>

#include <visualization_msgs/MarkerArray.h>

namespace apc_kinematics
{

class APCKinematics : public kinematics::KinematicsBase
{
public:
	APCKinematics();
	virtual ~APCKinematics();

	virtual bool initialize(const std::string& robot_description,
		const std::string& group_name,
		const std::string& base_frame,
		const std::string& tip_frame,
		double search_discretization) override;

	/**
	 * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
	 * @param ik_pose the desired pose of the link
	 * @param ik_seed_state an initial guess solution for the inverse kinematics
	 * @param solution the solution vector
	 * @param error_code an error code that encodes the reason for failure or success
	 * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
	 * @return True if a valid solution was found, false otherwise
	 */
	virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state,
		std::vector<double> &solution,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

	/**
	 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
	 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
	 * (or other numerical routines).
	 * @param ik_pose the desired pose of the link
	 * @param ik_seed_state an initial guess solution for the inverse kinematics
	 * @param timeout The amount of time (in seconds) available to the solver
	 * @param solution the solution vector
	 * @param error_code an error code that encodes the reason for failure or success
	 * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
	 * @return True if a valid solution was found, false otherwise
	 */
	virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
									const std::vector<double> &ik_seed_state,
									double timeout,
									std::vector<double> &solution,
									moveit_msgs::MoveItErrorCodes &error_code,
									const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

	/**
	 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
	 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
	 * (or other numerical routines).
	 * @param ik_pose the desired pose of the link
	 * @param ik_seed_state an initial guess solution for the inverse kinematics
	 * @param timeout The amount of time (in seconds) available to the solver
	 * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
	 * @param solution the solution vector
	 * @param error_code an error code that encodes the reason for failure or success
	 * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
	 * @return True if a valid solution was found, false otherwise
	 */
	virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
									const std::vector<double> &ik_seed_state,
									double timeout,
									const std::vector<double> &consistency_limits,
									std::vector<double> &solution,
									moveit_msgs::MoveItErrorCodes &error_code,
									const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

	/**
	 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
	 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
	 * (or other numerical routines).
	 * @param ik_pose the desired pose of the link
	 * @param ik_seed_state an initial guess solution for the inverse kinematics
	 * @param timeout The amount of time (in seconds) available to the solver
	 * @param solution the solution vector
	 * @param solution_callback A callback solution for the IK solution
	 * @param error_code an error code that encodes the reason for failure or success
	 * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
	 * @return True if a valid solution was found, false otherwise
	 */
	virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
									const std::vector<double> &ik_seed_state,
									double timeout,
									std::vector<double> &solution,
									const IKCallbackFn &solution_callback,
									moveit_msgs::MoveItErrorCodes &error_code,
									const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

	/**
	 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
	 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
	 * (or other numerical routines).
	 * @param ik_pose the desired pose of the link
	 * @param ik_seed_state an initial guess solution for the inverse kinematics
	 * @param timeout The amount of time (in seconds) available to the solver
	 * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
	 * @param solution the solution vector
	 * @param solution_callback A callback solution for the IK solution
	 * @param error_code an error code that encodes the reason for failure or success
	 * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
	 * @return True if a valid solution was found, false otherwise
	 */
	virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
									const std::vector<double> &ik_seed_state,
									double timeout,
									const std::vector<double> &consistency_limits,
									std::vector<double> &solution,
									const IKCallbackFn &solution_callback,
									moveit_msgs::MoveItErrorCodes &error_code,
									const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

	virtual bool getPositionFK(const std::vector<std::string> &link_names,
		const std::vector<double> &joint_angles,
		std::vector<geometry_msgs::Pose> &poses) const override;

	/**
	 * @brief  Return all the joint names in the order they are used internally
	 */
	virtual const std::vector<std::string>& getJointNames() const override
	{ return m_jointNames; }

	/**
	 * @brief  Return all the link names in the order they are represented internally
	 */
	virtual const std::vector<std::string>& getLinkNames() const
	{ return m_linkNames; }
private:
	void costFunction(int joint, double q, double seed_q, double* cost, double *costGradient) const;
	void wristCostFunction(rbdl_parser::URDF_RBDL_Model& model, const Eigen::VectorXd& currentJointAngles, double* cost, Eigen::VectorXd* costGradient) const;
	void tableCostFunction(rbdl_parser::URDF_RBDL_Model& model, const Eigen::VectorXd& currentJointAngles, double* cost, Eigen::VectorXd* costGradient) const;

	/**
	 * @return True if the given state is within the joint angle limits.
	 */
	bool isStateValid(std::vector<double> &state) const;
	void pruneToLimits(Eigen::VectorXd &state) const;

	void handleJointStates(const sensor_msgs::JointStateConstPtr& msg);
	
	boost::shared_ptr<urdf::Model> m_urdf;
	boost::shared_ptr<const srdf::Model> m_srdf;
	rbdl_parser::URDF_RBDL_Model m_rbdl;

	std::vector<int> m_links;
	std::vector<std::string> m_linkNames;
	std::vector<std::string> m_jointNames;
	std::vector<int> m_joints;
	std::vector<double> m_upperLimit;
	std::vector<double> m_lowerLimit;
	
	srdf::Model::GroupState m_convenient_state;
	srdf::Model::GroupState m_weighting;

	int m_tip;
	int m_base;

	RigidBodyDynamics::Math::SpatialTransform m_baseFixedTrans;
	RigidBodyDynamics::Math::SpatialTransform m_fixedTrans;
	
	config_server::Parameter<int> m_param_max_iterations;
	
	config_server::Parameter<float> m_param_max_posdiff;
	config_server::Parameter<float> m_param_angle_weight_slope;
	
	config_server::Parameter<bool> m_param_nullspace_optimization;
	
	config_server::Parameter<float> m_param_weight_convenient;
	config_server::Parameter<float> m_param_weight_seed;
	config_server::Parameter<float> m_param_weight_limit;
	
	config_server::Parameter<float> m_param_alpha;
	config_server::Parameter<float> m_param_alpha_decay;

	config_server::Parameter<bool> m_param_eef_enabled;
	config_server::Parameter<float> m_param_eef_soft_limit_x;
	config_server::Parameter<float> m_param_eef_soft_limit_z;
	config_server::Parameter<float> m_param_eef_s;
	config_server::Parameter<float> m_param_eef_orient;
	
	bool m_nullspaceOptimization;

	unsigned int m_ik_limit_link;
	unsigned int m_worldLink;
	unsigned int m_ik_wrist_link;
	ros::Subscriber m_sub_js;
	Eigen::VectorXd m_rbdlMeasuredJointAngles;
	ros::Publisher m_pub_elbowMarker;

	mutable ros::Publisher m_pub_limitMarkers;
	mutable visualization_msgs::MarkerArray m_limitMarkers;

	enum LimitMarker
	{
		LIMIT_X,
		LIMIT_Z,
		LIMIT_TABLE,
		LIMIT_COUNT
	};

	bool m_ignoreRoll;
};

}

#endif
