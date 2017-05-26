// Kinematics plugin for MoveIt! implementing null-space optimization
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "apc_kinematics.h"
#include "srdf_cache.h"

#include <urdf/model.h>

#include <ros/console.h>

#include <boost/make_shared.hpp>

#include <pluginlib/class_list_macros.h>

#include <rbdl/Kinematics.h>

#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

#include <angles/angles.h>

#include <math.h>

#include <exception>

#include <visualization_msgs/MarkerArray.h>

/** \brief Computes the jacobian
 *
 * \param model   	rigid body model
 * \param Q       	state vector of the joints
 * \param joint_mapping mapping from the joint ids to the rbdl joint ids
 * \param body_id 	the id of the body
 * \param base_id 	the id of the base
 * \param J       	a matrix where the result will be stored in
 *
 * \returns A 6 x \#dof_count matrix of the jacobian
 */
static void _CalcPointRotJacobian (RigidBodyDynamics::Model &model,
	const RigidBodyDynamics::Math::VectorNd &Q,
	const std::vector<int> &joint_mapping,
	unsigned int body_id,
	unsigned int base_id,
	RigidBodyDynamics::Math::MatrixNd &J)
{
	using namespace RigidBodyDynamics;
	using namespace Math;
	
	// Convert the joint vector into the rbdl convention
	Eigen::VectorXd rbdlJoints = Eigen::VectorXd::Zero(model.dof_count);
	for(unsigned int i = 0; i < joint_mapping.size(); ++i)
		rbdlJoints[joint_mapping[i]-1] = Q[i];

	Vector3d point_base_pos = CalcBodyToBaseCoordinates(model, rbdlJoints, body_id, Eigen::VectorXd::Zero(3), false);
	SpatialMatrix point_trans = Math::Xtrans_mat (point_base_pos);

	assert (J.rows() == 6 && J.cols() == (int)joint_mapping.size() );

	J.setZero();

	// we have to make sure that only the joints that contribute to the
	// bodies motion also get non-zero columns in the jacobian.
	// VectorNd e = VectorNd::Zero(Q.size() + 1);
	char *e = new char[model.dof_count + 1];
	if (e == NULL) {
		std::cerr << "Error: allocating memory." << std::endl;
		abort();
	}
	memset (&e[0], 0, model.dof_count + 1);

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
	}

	unsigned int j = reference_body_id;

	// e[j] is set to 1 if joint j contributes to the jacobian that we are
	// computing. For all other joints the column will be zero.
	
	while (j != 0) {
		e[j] = 1;
		j = model.lambda[j];
	}
	
	for (j = 1; j < model.mBodies.size(); j++) {
		if (e[j] == 1) {
			SpatialVector S_base;
			S_base = point_trans * spatial_inverse(model.X_base[j].toMatrix()) * model.S[j];

			// Map back from the rbdl indices
			int i = std::find(joint_mapping.begin(), joint_mapping.end(), j) - joint_mapping.begin();
					
			J.col(i) = S_base;
		}
	}
	
	delete[] e;
}

/**
 * Computes Selectively Damped Least Squares as described by Buss and Kim in:
 * 
 * Buss, Samuel R., and Jin-Su Kim. "Selectively damped least squares for inverse kinematics." journal of graphics, gpu, and game tools 10.3 (2005): 37-49.
 * 
 * \param jacobian   	the current jacobian
 * \param e       	the pose diff between the end effector and the desired target pose
 * \param modifiedJ 	a matrix where the modified jacobian will be stored in
 * \param invModifiedJ 	a matrix where the inverse of the modified jacobian will be stored in
 */
template<typename _Matrix_Type_>
void sdls(const _Matrix_Type_ &jacobian, Eigen::VectorXd &e, Eigen::MatrixXd &modifiedJ, Eigen::MatrixXd &invModifiedJ, double gamma_max = 0.78539816) //PI/4
{	
	unsigned int cols = jacobian.cols();
	unsigned int rows = jacobian.rows();
	
	// Calculate the norms of columns of the jacobian
	Eigen::VectorXd p(cols);
	for(unsigned int j = 0; j<cols; j++)
	{
		p(j) = jacobian.col(j).norm();
	}

	Eigen::JacobiSVD< _Matrix_Type_ > svd(jacobian ,Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::VectorXd alpha(rows);
	for(unsigned int i = 0; i<rows; i++)
	{
		alpha(i) = svd.matrixU().col(i).transpose() * e;
	}
	
	// Calculate the norms of U
	Eigen::VectorXd N(rows); // each component of N is 1
	for(unsigned int i = 0; i<rows; i++)
	{
		N(i) = svd.matrixU().col(i).norm();
	}
	
	Eigen::VectorXd M(rows);
	for(unsigned int i = 0; i<rows; i++)
	{
		if(std::abs(svd.singularValues()(i)) < 1.0e-4)
			continue;
		
		double tmp = 0.0;
		for(unsigned int j = 0; j<cols; j++)
		{
			tmp += std::abs(svd.matrixV()(j, i)) * p(j);
		}
		
		M(i) = (1.0/svd.singularValues()(i)) * tmp;
	}
	
	Eigen::VectorXd gamma(rows);
	gamma.setConstant(gamma_max);
	for(unsigned int i = 0; i<rows; i++)
	{
		if(std::abs(svd.singularValues()(i)) < 1.0e-4)
			continue;
		
		if(N(i)<M(i))
		{
			gamma(i) *= N(i)/M(i);
		}
	}
	
	modifiedJ.setZero();
	invModifiedJ.setZero();
	
	for(unsigned int i=0; i<rows; i++)
	{
		if(std::abs(svd.singularValues()(i)) < 1.0e-4)
			continue;
		
		Eigen::VectorXd phi_tmp = (1.0/svd.singularValues()(i)) * alpha(i) * svd.matrixV().col(i);
		double max = phi_tmp.maxCoeff();
		
		Eigen::MatrixXd modifiedJtmp(rows, cols);
		modifiedJtmp = svd.singularValues()(i) * svd.matrixU().col(i) * svd.matrixV().col(i).transpose();
		
		Eigen::MatrixXd invModifiedJtmp(cols, rows);
		invModifiedJtmp = (1.0/svd.singularValues()(i)) * svd.matrixV().col(i) * svd.matrixU().col(i).transpose();
		
		if(gamma(i) < max)
		{
			invModifiedJtmp *= (gamma(i)/max);	
			modifiedJtmp *= (max/gamma(i));
		}
		
		modifiedJ += modifiedJtmp;
		invModifiedJ += invModifiedJtmp;
	}
}


namespace apc_kinematics
{

APCKinematics::APCKinematics() 
 : m_param_max_iterations("/ik/max_iterations", 1, 1, 100, 20)
 , m_param_max_posdiff("/ik/max_posdiff", 0.0, 0.01, 1.0, 0.1)
 , m_param_angle_weight_slope("/ik/angle_weight_slope", -10.0, 0.1, 0.0, -4.0)
 , m_param_nullspace_optimization("/ik/nullspace_optimization", true)
 , m_param_weight_convenient("/ik/cost/weight_convenient", 0.0, 0.1, 10.0, 1.0)
 , m_param_weight_seed("/ik/cost/weight_seed", 0.0, 0.1, 10.0, 1.0)
 , m_param_weight_limit("/ik/cost/weight_limit", 0.0, 0.1, 10.0, 1.0)
 , m_param_alpha("/ik/alpha", 0.0, 0.01, 1.0, 0.05)
 , m_param_alpha_decay("/ik/alpha_decay", 0.0, 0.01, 1.0, 0.9)
 , m_param_eef_enabled("/ik/eef/enabled", true)
 , m_param_eef_soft_limit_x("/ik/eef/soft_limit/x", 0.0, 0.001, 1.5, 0.005)
 , m_param_eef_soft_limit_z("/ik/eef/soft_limit/z", 0.0, 0.001, 1.5, 0.005)
 , m_param_eef_s("/ik/eef/s", 0.0, 0.001, 5000.0, 0.5)
 , m_param_eef_orient("/ik/eef/orient", 0.0, 0.001, 5000.0, 0.5)
 , m_ik_limit_link(-1)
{
}

APCKinematics::~APCKinematics()
{
}

bool APCKinematics::initialize(const std::string& robot_description,
	const std::string& group_name,
	const std::string& base_frame,
	const std::string& tip_frame,
	double search_discretization)
{
	ROS_INFO("IK base frame: %s", base_frame.c_str());
	setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

	// Robot model (from URDF model on parameter server)
	m_urdf = boost::make_shared<urdf::Model>();
	if(!m_urdf->initParam(robot_description))
	{
		ROS_ERROR("Could not get URDF model");
		return false;
	}

	m_rbdl.initFrom(*m_urdf, base_frame);
	
	m_srdf = loadSRDF();
	
	// ... and search for the convenient state and the weighting
	bool convenient_state_found = false;
	bool weighting_found = false;
	std::vector<srdf::Model::GroupState> groupStates = m_srdf->getGroupStates();
	for(unsigned int i=0; i<groupStates.size(); ++i)
	{
		if(groupStates[i].group_ == group_name && groupStates[i].name_.substr(0, 10) == "convenient")
		{
			m_convenient_state = groupStates[i];
			convenient_state_found = true;
		}
		else if(groupStates[i].group_ == group_name && groupStates[i].name_.substr(0, 9) == "weighting")
		{
			m_weighting = groupStates[i];
			weighting_found = true;
		}
	}
	
	// we can only use the nullspace opimization if these states exist
	m_nullspaceOptimization = weighting_found && convenient_state_found;
	if(!m_nullspaceOptimization)
	{
		ROS_WARN_STREAM("Unable to find convenient state or weighting for group "<<group_name<<". Disabling nullspace optimization!");	
	}

	unsigned int base_id = m_rbdl.GetBodyId(base_frame.c_str());
	if(base_id == (unsigned int)-1)
	{
		ROS_ERROR("Could not find base frame '%s'", base_frame.c_str());
		return false;
	}

	unsigned int tip = m_rbdl.GetBodyId(tip_frame.c_str());
	if(tip == (unsigned int)-1)
	{
		ROS_ERROR("Could not find tip_frame '%s'", tip_frame.c_str());
		return false;
	}

	if(tip >= m_rbdl.fixed_body_discriminator)
	{
		m_fixedTrans = m_rbdl.mFixedBodies[tip - m_rbdl.fixed_body_discriminator].mParentTransform;
		m_tip = m_rbdl.mFixedBodies[tip - m_rbdl.fixed_body_discriminator].mMovableParent;
	}
	else
	{
		m_fixedTrans.E.setIdentity();
		m_fixedTrans.r.setZero();
		m_tip = tip;
	}

	do
	{
		m_links.push_back(tip);
		m_linkNames.push_back(m_rbdl.GetBodyName(tip));

		if(tip < m_rbdl.fixed_body_discriminator)
		{
			m_joints.push_back(tip);
			m_jointNames.push_back(m_rbdl.jointName(tip));

			boost::shared_ptr<const urdf::Joint> urdfJoint = m_urdf->getJoint(m_jointNames.back());

			double epsilon;
			if(urdfJoint->type == urdf::Joint::PRISMATIC)
				epsilon = 0.001;
			else
				epsilon = 2.0 * (M_PI / 180.0);

			m_upperLimit.push_back(urdfJoint->limits->upper - epsilon);
			m_lowerLimit.push_back(urdfJoint->limits->lower + epsilon);

			ROS_DEBUG("Solver: Link '%s' with parent joint '%s' (limits: %f to %f)",
				m_linkNames.back().c_str(),
				m_jointNames.back().c_str(),
				m_lowerLimit.back(), m_upperLimit.back()
			);

			tip = m_rbdl.lambda[tip];
		}
		else
		{
			ROS_DEBUG("Fixed transform link '%s'", m_linkNames.back().c_str());
			tip = m_rbdl.mFixedBodies[tip - m_rbdl.fixed_body_discriminator].mMovableParent;
		}
	}
	while(tip != 0);

	m_rbdlMeasuredJointAngles = Eigen::VectorXd::Zero(m_rbdl.dof_count);
	m_sub_js = ros::NodeHandle().subscribe("/joint_states", 1,
		&APCKinematics::handleJointStates, this
	);

	// Disable eef nullspace for arm-only IK
	if(group_name.find("arm_with_eef") != std::string::npos)
	{
		m_worldLink = m_rbdl.GetBodyId("world");
		m_ik_limit_link = m_rbdl.GetBodyId("ik_limit_link");
		m_ik_wrist_link = m_rbdl.GetBodyId("wrist_2_link");
	}
	else
	{
		m_worldLink = m_ik_limit_link =  std::numeric_limits<unsigned int>::max();
		m_ik_wrist_link = std::numeric_limits<unsigned int>::max();
	}

	ROS_INFO("world link: %u, ik finger link: %u", m_worldLink, m_ik_limit_link);

	m_pub_elbowMarker = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
		"/ik/" + group_name + "/elbow", 1
	);

	std::string ignoreRoll_param = "/robot_description_kinematics/" + group_name + "/apc_kinematics_ignore_roll";
	ros::NodeHandle nh;
	nh.param(ignoreRoll_param, m_ignoreRoll, false);

	ROS_INFO("Group '%s' is %s roll.", group_name.c_str(), m_ignoreRoll ? "ignoring" : "not ignoring");

	m_limitMarkers.markers.resize(LIMIT_COUNT);
	m_limitMarkers.markers[LIMIT_X].type = visualization_msgs::Marker::CUBE;
	m_limitMarkers.markers[LIMIT_X].header.frame_id = "world";
	m_limitMarkers.markers[LIMIT_X].id = LIMIT_X;
	m_limitMarkers.markers[LIMIT_X].scale.x = 0.01;
	m_limitMarkers.markers[LIMIT_X].scale.y = 5.0;
	m_limitMarkers.markers[LIMIT_X].scale.z = 5.0;
	m_limitMarkers.markers[LIMIT_X].color.r = 1.0;
	m_limitMarkers.markers[LIMIT_X].color.a = 0.3;
	m_limitMarkers.markers[LIMIT_X].pose.orientation.w = 1.0;

	m_limitMarkers.markers[LIMIT_Z].type = visualization_msgs::Marker::CUBE;
	m_limitMarkers.markers[LIMIT_Z].header.frame_id = "world";
	m_limitMarkers.markers[LIMIT_Z].id = LIMIT_Z;
	m_limitMarkers.markers[LIMIT_Z].scale.x = 5.0;
	m_limitMarkers.markers[LIMIT_Z].scale.y = 5.0;
	m_limitMarkers.markers[LIMIT_Z].scale.z = 0.01;
	m_limitMarkers.markers[LIMIT_Z].color.b = 1.0;
	m_limitMarkers.markers[LIMIT_Z].color.a = 0.3;
	m_limitMarkers.markers[LIMIT_Z].pose.orientation.w = 1.0;

	m_limitMarkers.markers[LIMIT_TABLE].type = visualization_msgs::Marker::CUBE;
	m_limitMarkers.markers[LIMIT_TABLE].header.frame_id = "base_link";
	m_limitMarkers.markers[LIMIT_TABLE].id = LIMIT_TABLE;
	m_limitMarkers.markers[LIMIT_TABLE].scale.x = 0.01;
	m_limitMarkers.markers[LIMIT_TABLE].scale.y = 5.0;
	m_limitMarkers.markers[LIMIT_TABLE].scale.z = 5.0;
	m_limitMarkers.markers[LIMIT_TABLE].color.b = 1.0;
	m_limitMarkers.markers[LIMIT_TABLE].color.a = 0.3;

	m_pub_limitMarkers = nh.advertise<visualization_msgs::MarkerArray>("limit_markers", 1);

	return true;
}

bool APCKinematics::getPositionIK(const geometry_msgs::Pose &ik_pose,
	const std::vector<double> &ik_seed_state,
	std::vector<double> &solution,
	moveit_msgs::MoveItErrorCodes &error_code,
	const kinematics::KinematicsQueryOptions &options) const
{
	ROS_INFO("1");
	return false;
}

bool APCKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
	const std::vector<double> &ik_seed_state,
	double timeout,
	std::vector<double> &solution,
	moveit_msgs::MoveItErrorCodes &error_code,
	const kinematics::KinematicsQueryOptions &options) const
{
	ROS_INFO("2");
	return false;
}

static Eigen::Vector3d getEuler(const Eigen::Matrix3d& R)
{
// 	Eigen::Vector3d e = mat.eulerAngles(2,1,0);
// 	return Eigen::Vector3d(e[2], e[1], e[0]);
// 	return mat.eulerAngles(0,1,2);

	Eigen::Vector3d p;

	// euler angles
	p( 1 ) = atan2( -R( 2, 0 ), sqrtf( R( 0, 0 )*R( 0, 0 ) + R( 1, 0 )*R( 1, 0 ) ) );
	if( fabs( p( 1 ) - 0.5*M_PI ) < 1e-5 ) {
		p( 2 ) = 0;
		p( 0 ) = atan2( R( 0, 1 ), R( 1, 1 ) );
	}
	else if( fabs( p( 1 ) + 0.5*M_PI ) < 1e-5 ) {
		p( 2 ) = 0;
		p( 0 ) = -atan2( R( 0, 1 ), R( 1, 1 ) );
	}
	else {
		p( 0 ) = atan2( R( 2, 1 ) / cos( p( 1 ) ), R( 2, 2 ) / cos( p( 1 ) ) );
		p( 2 ) = atan2( R( 1, 0 ) / cos( p( 1 ) ), R( 0, 0 ) / cos( p( 1 ) ) );
	}

	return p;
}

static Eigen::VectorXd calcPoseDiff(const Eigen::Affine3d& current, const Eigen::Affine3d& target, bool ignoreXRotation)
{
	Eigen::VectorXd ret(6);

	if(ignoreXRotation)
	{
		// Find minimum rotation to rotate target.x onto current.x
		Eigen::Vector3d axis = target.rotation().col(0).cross(current.rotation().col(0));
		double norm = axis.norm();
		if(norm < 1e-10)
			ret.head<3>().setZero();
		else
		{
			double angle = asin(norm);
			axis.normalize();

			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(angle, axis);

			ret.head<3>() = getEuler(rot);
		}
	}
	else
	{
		ret.head<3>() = getEuler(current.rotation() * target.rotation().transpose());
	}

	ret.tail<3>() = current.translation() - target.translation();

	return ret;
}

void APCKinematics::costFunction(int joint, double q, double seed_q, double* cost, double *costGradient) const
{
	double convenient = m_convenient_state.joint_values_.at(m_jointNames[joint])[0];
	double weight = m_weighting.joint_values_.at(m_jointNames[joint])[0];
	double angleDiff = 0.1 * (q - convenient);
	angleDiff *= pow(weight, 2.0);
	
	double diffFromSeed = q - seed_q;

	const double limitStartDiff = 0.1 * M_PI;
	const double limitSlope = 10.0;
	double maxJointAngleOffset = 0;
	double minJointAngleOffset = 0;
	double maxJointAngleOffsetGradient = 0;
	double minJointAngleOffsetGradient = 0;

	double maxJointAngleDiff = q - (m_upperLimit[joint] - limitStartDiff);
	if(maxJointAngleDiff > 0)
	{
		maxJointAngleOffset = limitSlope * maxJointAngleDiff * maxJointAngleDiff;
		maxJointAngleOffsetGradient = 2.0 * limitSlope * maxJointAngleDiff;

// 		if(q > m_upperLimit[joint])
// 			ROS_INFO("Joint limit violation %s (%f, upper limit %f) => cost %f", m_jointNames[joint].c_str(), q, m_upperLimit[joint], maxJointAngleOffset);
	}

	double minJointAngleDiff = q - (m_lowerLimit[joint] + limitStartDiff);
	if(minJointAngleDiff < 0)
	{
		minJointAngleOffset = limitSlope * minJointAngleDiff * minJointAngleDiff;
		minJointAngleOffsetGradient = 2.0 * limitSlope * minJointAngleDiff;

// 		if(q < m_lowerLimit[joint])
// 			ROS_INFO("Joint limit violation %s (%f, lower limit %f) => cost %f", m_jointNames[joint].c_str(), q, m_lowerLimit[joint], minJointAngleOffset);
	}

	*cost += m_param_weight_convenient()*angleDiff*angleDiff
		+ m_param_weight_seed()*diffFromSeed*diffFromSeed 
		+ m_param_weight_limit()*(maxJointAngleOffset + minJointAngleOffset);

	*costGradient += m_param_weight_convenient() * 2.0 * angleDiff
			+ m_param_weight_seed()* 2.0 * diffFromSeed 
			+ m_param_weight_limit()*(maxJointAngleOffsetGradient + minJointAngleOffsetGradient);
}


void APCKinematics::wristCostFunction(rbdl_parser::URDF_RBDL_Model& model, const Eigen::VectorXd& currentJointAngles, double* cost, Eigen::VectorXd* costGradient) const
{
	Eigen::MatrixXd Jwrist(6, m_joints.size());

	_CalcPointRotJacobian(model, currentJointAngles, m_joints, m_ik_limit_link, m_base, Jwrist);

	// We are only interested in the position (bottom three rows)
// 	Jwrist = Jwrist.bottomRows<3>();

	RigidBodyDynamics::Math::SpatialTransform eefTransform;

	if (m_ik_limit_link >= model.fixed_body_discriminator) {
		auto fBody = m_rbdl.mFixedBodies[m_ik_limit_link - m_rbdl.fixed_body_discriminator];
		auto fixedTransform = fBody.mParentTransform;
		auto parentPose = model.X_base[fBody.mMovableParent];

		eefTransform.E = parentPose.E.transpose() * fixedTransform.E.transpose();
		eefTransform.r = parentPose.r + parentPose.E.transpose() * fixedTransform.r;
	}
	else
	{
		eefTransform.E = model.X_base[m_ik_limit_link].E.transpose();
		eefTransform.r = model.X_base[m_ik_limit_link].r;
	}

	Eigen::Vector3d eef = eefTransform.r;

	RigidBodyDynamics::Math::SpatialTransform worldTransform;

	// TODO: Can we extract this piece of code? It is used everywhere.
	if((unsigned int)m_worldLink >= model.fixed_body_discriminator)
	{
		auto fBody = m_rbdl.mFixedBodies[m_worldLink - m_rbdl.fixed_body_discriminator];
		auto fixedTransform = fBody.mParentTransform;
		auto parentPose = model.X_base[fBody.mMovableParent];

		worldTransform.E = fixedTransform.E * parentPose.E;
		worldTransform.r = parentPose.r + parentPose.E.transpose() * fixedTransform.r;
	}
	else
		worldTransform = model.X_base[m_worldLink];

	// Transform elbow and Jelbow into base_link
	eef = RigidBodyDynamics::CalcBaseToBodyCoordinates(
		model, Eigen::VectorXd(), m_worldLink, eef, false
	);
	Jwrist.topRows<3>() = worldTransform.E * Jwrist.topRows<3>();
	Jwrist.bottomRows<3>() = worldTransform.E * Jwrist.bottomRows<3>();

	visualization_msgs::MarkerArray markers;
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time::now();

		marker.action = marker.ADD;

		marker.id = 0;
		marker.type = marker.SPHERE;

		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;

		marker.pose.position.x = eef.x();
		marker.pose.position.y = eef.y();
		marker.pose.position.z = eef.z();

		marker.pose.orientation.w = 1.0;

		marker.color.a = 1.0;
		marker.color.r = 1.0;

		markers.markers.push_back(marker);
	}

	m_pub_elbowMarker.publish(markers);

	double colCost = 0;
	Eigen::VectorXd cost_gradient(m_joints.size());
	cost_gradient.setZero();

	m_limitMarkers.markers[LIMIT_X].pose.position.x = m_param_eef_soft_limit_x();
	m_limitMarkers.markers[LIMIT_Z].pose.position.z = m_param_eef_soft_limit_z();

	if(eef.z() < m_param_eef_soft_limit_z())
	{
		double violation = eef.z() - m_param_eef_soft_limit_z();

		// translation
		colCost += m_param_eef_s() * pow(violation, 2);

		// fifth row of Jwrist: z component
		cost_gradient += (2.0 * m_param_eef_s() * violation) * Jwrist.row(5);

		// orientation
// 		double pitch = getEuler(eefTransform.E)[1];
// 		colCost += m_param_eef_orient() * pow(pitch, 2);
// 		cost_gradient += (2.0 * m_param_eef_orient() * pitch) * Jwrist.row(5);
	}

	if(eef.x() > m_param_eef_soft_limit_x())
	{
		double violation = eef.x() - m_param_eef_soft_limit_x();

		// translation
		colCost += m_param_eef_s() * pow(violation, 2);

		// third row of Jwrist: x component
		cost_gradient += (2.0 * m_param_eef_s() * violation) * Jwrist.row(3);

		// orientation
// 		double pitch = getEuler(eefTransform.E)[1];
// 		colCost += m_param_eef_orient() * pow(pitch, 2);
// 		cost_gradient += (2.0 * m_param_eef_orient() * pitch) * Jwrist.row(3);
	}

	*cost += colCost;
	*costGradient += cost_gradient;
}

void APCKinematics::tableCostFunction(rbdl_parser::URDF_RBDL_Model& model, const Eigen::VectorXd& currentJointAngles, double* cost, Eigen::VectorXd* costGradient) const
{
	Eigen::MatrixXd Jwrist(6, m_joints.size());

	_CalcPointRotJacobian(model, currentJointAngles, m_joints, m_ik_wrist_link, m_base, Jwrist);

	// We are only interested in the position (bottom three rows)
// 	Jwrist = Jwrist.bottomRows<3>();

	RigidBodyDynamics::Math::SpatialTransform eefTransform;

	if (m_ik_wrist_link >= model.fixed_body_discriminator) {
		auto fBody = m_rbdl.mFixedBodies[m_ik_wrist_link - m_rbdl.fixed_body_discriminator];
		auto fixedTransform = fBody.mParentTransform;
		auto parentPose = model.X_base[fBody.mMovableParent];

		eefTransform.E = parentPose.E.transpose() * fixedTransform.E.transpose();
		eefTransform.r = parentPose.r + parentPose.E.transpose() * fixedTransform.r;
	}
	else
	{
		eefTransform.E = model.X_base[m_ik_wrist_link].E.transpose();
		eefTransform.r = model.X_base[m_ik_wrist_link].r;
	}

	Eigen::Vector3d eef = eefTransform.r;

	constexpr double planeAngle = 40.0 * M_PI / 180.0;
	Eigen::Vector3d planeNormal(0.0, cos(planeAngle), sin(planeAngle));

	double planeDist = 0.45;

	{
		Eigen::Matrix3d rot;
		rot.col(0) = planeNormal;
		rot.col(1) = -Eigen::Vector3d::UnitX();
		rot.col(2) = rot.col(0).cross(rot.col(1));

		Eigen::Affine3d pose;
		pose = Eigen::Translation3d(planeNormal * planeDist) * rot;

		tf::poseEigenToMsg(pose, m_limitMarkers.markers[LIMIT_TABLE].pose);
	}

	ROS_INFO_STREAM("eef: " << eef.transpose());

	double violation = -planeNormal.dot(eef) + planeDist;

	double colCost = 0;
	Eigen::VectorXd cost_gradient(m_joints.size());
	cost_gradient.setZero();

	ROS_INFO("violation: %f", violation);

	if(violation > 0.0)
	{
		colCost += m_param_eef_s() * pow(violation, 2);
		cost_gradient += -(2.0 * m_param_eef_s() * violation) * (planeNormal.transpose() * Jwrist.bottomRows<3>());
	}

	*cost += colCost;
	*costGradient += cost_gradient;
}


bool APCKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
	const std::vector<double> &ik_seed_state,
	double timeout,
	const std::vector<double> &consistency_limits,
	std::vector<double> &solution,
	moveit_msgs::MoveItErrorCodes &error_code,
	const kinematics::KinematicsQueryOptions &options) const
{
	ROS_DEBUG("IK start");
	ros::Time startTime = ros::Time::now();

	rbdl_parser::URDF_RBDL_Model model = m_rbdl;

	const double convergence_threshold_posdiff = 1e-6;
	const double convergence_threshold_anglediff = 1e-6;
	const double convergence_threshold_costdiff = 1e-6;
	const double alpha = m_param_alpha();
	const int max_iterations = m_param_max_iterations();
	bool nullspace_optimization = m_nullspaceOptimization && m_param_nullspace_optimization();
	float max_posdiff = m_param_max_posdiff();
// 	float angle_weight_slope = m_param_angle_weight_slope();
	
	int numJoints = m_joints.size();
	
	Eigen::VectorXd startJointAngles = Eigen::VectorXd::Zero(numJoints);
	for(unsigned int i = 0; i < ik_seed_state.size(); ++i)
		startJointAngles[i] = ik_seed_state[i];
	
	Eigen::VectorXd currentJointAngles = startJointAngles;
	Eigen::Affine3d currentEndEffectorPose;
	
	Eigen::Affine3d targetEndEffectorPose;
	{
		Eigen::Affine3d pose;
		tf::poseMsgToEigen(ik_pose, targetEndEffectorPose);
	}

	Eigen::VectorXd poseDiff;
	double posdiff;
	double anglediff;
	double lastCost = 0;
	double cost = 0;
	Eigen::VectorXd costGradient = Eigen::VectorXd::Zero(numJoints);
	double costdiff;

	int iteration = 0;
	bool singular = false;

	bool haveEEF = m_param_eef_enabled()
			&& m_ik_limit_link < std::numeric_limits<unsigned int>::max()
			&& m_worldLink < std::numeric_limits<unsigned int>::max();

	while(iteration < max_iterations)
	{
		// Copy the current joint angles into rbdl convention and update the model
		// start at the current measured joint angles for the joints that we
		// do not control in this solver.
		Eigen::VectorXd tmpJoints = m_rbdlMeasuredJointAngles;
		for(unsigned int i = 0; i < m_joints.size(); ++i)
			tmpJoints[m_joints[i]-1] = currentJointAngles[i];
		RigidBodyDynamics::UpdateKinematicsCustom(model, &tmpJoints, 0, 0);

		// Calculate the current end effector pose...
		const auto& X = model.X_base[m_tip];
		currentEndEffectorPose.setIdentity();
		currentEndEffectorPose.translate(X.r + X.E.transpose() * m_fixedTrans.r);
		currentEndEffectorPose.rotate(X.E.transpose() * m_fixedTrans.E.transpose());

		poseDiff = calcPoseDiff(currentEndEffectorPose, targetEndEffectorPose, m_ignoreRoll);
		
		posdiff = poseDiff.tail<3>().norm();
		anglediff = poseDiff.head<3>().norm();

		// Weighting matrix for poseDiff
		Eigen::MatrixXd W = Eigen::MatrixXd::Identity(6, 6);
		
		if(posdiff > max_posdiff)
		{
			// Limit the maximal translational posediff
			poseDiff.tail<3>() *= (max_posdiff/posdiff);
			
// 			double angle_weight =  std::max(1 + angle_weight_slope * (posdiff-max_posdiff), 0.0);
// 			
// 			W(0,0) = angle_weight;
// 			W(1,1) = angle_weight;
// 			W(2,2) = angle_weight;
		}
		
		poseDiff = W * poseDiff;

		cost = 0;
		costGradient.setZero();
		bool nullSpaceActive = false;

		if(haveEEF)
		{
			wristCostFunction(model, currentJointAngles, &cost, &costGradient);
			tableCostFunction(model, currentJointAngles, &cost, &costGradient);

			nullSpaceActive = true;
		}

		if(nullspace_optimization)
		{
			// Calculate the costs for the current state
			for(unsigned int i = 0; i < m_joints.size(); ++i)
				costFunction(i, currentJointAngles[i], startJointAngles[i], &cost, &costGradient[i]);

			nullSpaceActive = true;
		}

		costdiff = std::abs(lastCost - cost);

		if( posdiff < convergence_threshold_posdiff
		 && anglediff < convergence_threshold_anglediff
		 && costdiff < convergence_threshold_costdiff)
		{
			break;
		}

		lastCost = cost;

		Eigen::MatrixXd J(6, numJoints);
		_CalcPointRotJacobian(model, currentJointAngles, m_joints, m_links.front(), m_base, J);

		if(m_ignoreRoll)
		{
			// remove the local roll from the jacobian to allow null space optimization to use it

			// convert rotation part into local frame
			J.topRows<3>() = targetEndEffectorPose.rotation().transpose() * J;

			// zero roll
			J.row(0).setZero();

			// convert back into global frame
			J.topRows<3>() = targetEndEffectorPose.rotation() * J;
		}

		J = W*J;
		
		Eigen::MatrixXd modifiedJ(6, numJoints);
		Eigen::MatrixXd invModifiedJ(numJoints, 6);
		
		sdls(J, poseDiff, modifiedJ, invModifiedJ);
		
		if(invModifiedJ.maxCoeff() > 1e4 || invModifiedJ.minCoeff() < -1e4)
		{
			ROS_WARN("invModifiedJ nearly singular");
			iteration = max_iterations;
			singular = true;
			break;
		}
		
		if(modifiedJ.maxCoeff() > 1e4 || modifiedJ.minCoeff() < -1e4)
		{
			ROS_WARN("modifiedJ nearly singular");
			iteration = max_iterations;
			singular = true;
			break;
		}

 		Eigen::VectorXd thetaDiff = invModifiedJ * poseDiff;
		
		if(nullSpaceActive)
		{
			Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(numJoints, numJoints);
			
			// Since the inverse of the jacobian is computed with some damping, the correct nullspace needs also to be computed with a similarly modified jacobian
			nullspace = nullspace - invModifiedJ * modifiedJ;

			//decrease the step size in each iteration
			double stepsize = alpha * pow(m_param_alpha_decay(), iteration);
			
			thetaDiff += stepsize * nullspace * costGradient;
		}
		
		currentJointAngles -= thetaDiff;

		iteration++;
		
		pruneToLimits(currentJointAngles);
	}

	if(singular) // This should never happen
	{
		ROS_INFO("No solution found :-(");
		return false;
	}

	solution.resize(m_joints.size());

	// This should never happen
	for(unsigned int i = 0; i < m_joints.size(); ++i)
	{
		if(!std::isfinite(currentJointAngles[i]))
		{
			ROS_ERROR("NaN in kinematics on joint %d, reporting failure...", i);
			return false;
		}
		solution[i] = currentJointAngles[i];
	}

 	ROS_DEBUG_STREAM("IK end (" << iteration << " iter)."/* Solutions: " << currentJointAngles.transpose()*/);
	
	ROS_DEBUG_STREAM("IK end, posdiff: "<<posdiff<<" , anglediff: "<<anglediff);
	
	ros::Time endTime = ros::Time::now();
       
	ros::Duration ik_time = endTime - startTime;
	ROS_DEBUG_STREAM("IK end, ik_time in millis: "<<(ik_time.toSec()*1000.0));

	m_pub_limitMarkers.publish(m_limitMarkers);

	
	//TODO limit the maximal amount the joint angles are allowed to differ from the start angles

	return true;
}

void APCKinematics::pruneToLimits(Eigen::VectorXd &state) const
{
	for(unsigned int i = 0; i< m_joints.size(); ++i)
	{
		if(state[i] > m_upperLimit[i])
		{
			state[i] = m_upperLimit[i];
		}
		else if(state[i] < m_lowerLimit[i]) 
		{
			state[i] = m_lowerLimit[i];
		}
	}
}

bool APCKinematics::isStateValid(std::vector<double> &state) const
{
	bool result = true;
	for(unsigned int i = 0; i< m_joints.size(); ++i) 
	{
		if(state[i] > m_upperLimit[i] || state[i] < m_lowerLimit[i]) 
		{
			result = false;
			break;
		}
	}
	
	return result;
}

bool APCKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
	const std::vector<double> &ik_seed_state,
	double timeout,
	std::vector<double> &solution,
	const IKCallbackFn &solution_callback,
	moveit_msgs::MoveItErrorCodes &error_code,
	const kinematics::KinematicsQueryOptions &options) const
{
	ROS_INFO("4");
	return false;
}

bool APCKinematics::searchPositionIK(const geometry_msgs::Pose &ik_pose,
	const std::vector<double> &ik_seed_state,
	double timeout,
	const std::vector<double> &consistency_limits,
	std::vector<double> &solution,
	const IKCallbackFn &solution_callback,
	moveit_msgs::MoveItErrorCodes &error_code,
	const kinematics::KinematicsQueryOptions &options) const
{
	ROS_INFO("5");
	return false;
}

bool APCKinematics::getPositionFK(const std::vector<std::string> &link_names,
	const std::vector<double> &joint_angles,
	std::vector<geometry_msgs::Pose> &poses) const
{
	rbdl_parser::URDF_RBDL_Model model = m_rbdl;

	if(joint_angles.size() != m_joints.size())
	{
		ROS_ERROR("Got invalid joint_angles");
		return false;
	}

	Eigen::VectorXd q(m_rbdl.dof_count);

	for(unsigned int i = 0; i < m_joints.size(); ++i)
	{
		q[m_joints[i]-1] = joint_angles[i];
	}

	RigidBodyDynamics::UpdateKinematicsCustom(model, &q, 0, 0);

	poses.resize(link_names.size());
	for(unsigned int i = 0; i < link_names.size(); ++i)
	{
		unsigned int id = model.GetBodyId(link_names[i].c_str());
		if(id == (unsigned int)-1)
		{
			ROS_ERROR("Invalid link '%s' requested for forward kinematics",
				link_names[i].c_str()
			);
			return false;
		}

		RigidBodyDynamics::Math::SpatialTransform X = model.X_base[id];

		Eigen::Affine3d pose;
		pose.translate(X.r);
		pose.rotate(X.E.transpose());

		tf::poseEigenToMsg(pose, poses[i]);
	}

	return true;
}

void APCKinematics::handleJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
	if(msg->name.size() != msg->position.size())
		return;

	for(size_t i = 0; i < msg->name.size(); ++i)
	{
		int index = m_rbdl.findJointIndex(msg->name[i]);
		if(index < 0)
			continue;

		m_rbdlMeasuredJointAngles[index-1] = msg->position[i];
	}
}

}

PLUGINLIB_EXPORT_CLASS(apc_kinematics::APCKinematics, kinematics::KinematicsBase)
