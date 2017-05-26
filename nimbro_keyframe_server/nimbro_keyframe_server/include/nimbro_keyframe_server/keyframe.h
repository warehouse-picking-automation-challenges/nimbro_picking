// A single keyframe in endeffector space
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_KEYFRAME_SERVER_KEYFRAME_H
#define NIMBRO_KEYFRAME_SERVER_KEYFRAME_H

#include <Eigen/Geometry>

#include <nimbro_keyframe_server/KeyFrameMsg.h>
#include <nimbro_keyframe_server/JointPosition.h>
#include <nimbro_keyframe_server/JointGroupMsg.h>

namespace YAML { class Emitter; class Node; }

namespace nimbro_keyframe_server
{

class KeyFrame
{
public:
	enum InterpolationSpace
	{
		IS_CARTESIAN,  //!< Interpolate in 3D cartesian space
		IS_JOINT_SPACE, //!< Interpolate in joint space
		IS_NONE, //!< Don't interpolate this joint group
	};


	class JointGroup
	{
	public:
		JointGroup();
		JointGroup(InterpolationSpace interpolation_space);
		JointGroup(InterpolationSpace interpolation_space,const Eigen::Affine3d& state);

		//! get current iterpolation space
		inline InterpolationSpace interpolationSpace() const
		{return m_interpolation_space;}

		//! get current invers kinematic state
		inline const Eigen::Affine3d& state() const
		{return m_state;}
		
		//! set the interpolation space for joint group
		void setInterpolationSpace(InterpolationSpace space);

		//! set the invers kinematic state for joint group
		void setState(const Eigen::Affine3d& state);
		
	private:
		InterpolationSpace m_interpolation_space;
		Eigen::Affine3d m_state;
	};

	typedef std::map<std::string, double> JointMap;
	typedef std::map<std::string, JointGroup> GroupMap;

	KeyFrame();

	//! set a label to identify the frame
	void setLabel(const std::string& label);

	//! a label used to identify the frame
	inline const std::string& label() const
	{ return m_label; }

	//! angular velocity limit for the interpolation
	void setAngularVelocity(double vel);

	//! angular velocity limit for the interpolation
	double angularVelocity() const
	{ return m_angularVelocity; }

	//! torque (as proportion) for all active joints in this keyframe
	void setTorqueProportion(double torqueProportion);

	//! torque (as proportion) for all active joints in this keyframe
	double torqueProportion() const
	{return m_torqueProportion;};

	InterpolationSpace interpolationSpace(std::string groupName) const
	{return m_jointGroups.at(groupName).interpolationSpace();}

	const Eigen::Affine3d& state(std::string groupName) const
	{return m_jointGroups.at(groupName).state();}

	//! linear velocity limit for the interpolation
	void setLinearVelocity(double vel) __attribute__((deprecated));

	//! linear velocity limit for the interpolation
	double linearVelocity() const
	{ return m_linearVelocity; }

	void setJointSpaceVelocity(double vel) {m_jointSpaceVelocity = vel;}
	double jointSpaceVelocity() const {return m_jointSpaceVelocity;}

	void setLinearAcceleration(double vel) {m_linearAcceleration = vel;}
	double linearAcceleration() const {return m_linearAcceleration; }

	void setAngularAcceleration(double vel) {m_angularAcceleration = vel;}
	double angularAcceleration() const {return m_angularAcceleration; }

	void setJointSpaceAcceleration(double vel) {m_jointSpaceAcceleration = vel;}
	double jointSpaceAcceleration() const {return m_jointSpaceAcceleration; }

	void setCartLinearVelocity(double vel) {m_linearVelocity = vel; }


	const JointMap& jointPositions() const
	{ return m_jointPositions; }


	inline const GroupMap& jointGroups() const
	{return m_jointGroups;}

	void setState(std::string groupName, const Eigen::Affine3d& state);

	void setInterpolationSpace(std::string groupName, InterpolationSpace space);

	void addJointGroup(std::string groupName, JointGroup group);

	bool hasJointGroup(std::string groupName) const;

	void setJointPosition(std::string name, double position);
	
	void removeJointPositions(std::vector<std::string>& jointNames);

	void deleteJointPositions()
	{ m_jointPositions = JointMap(); }


	void loadYAML (const YAML::Node& ,int vers = 1);

	KeyFrameMsg toMsg() const;

	static KeyFrame fromMsg(const KeyFrameMsg& msg);
	
	inline const bool continuous() const
	{return m_continuous;}

	void setContinuous(const bool continuous);
	
private:
	std::string m_label;
	double m_linearVelocity;
	double m_angularVelocity;
	double m_jointSpaceVelocity;
	double m_linearAcceleration;
	double m_angularAcceleration;
	double m_jointSpaceAcceleration;
	double m_torqueProportion;
	double m_gripperAngle;
	double m_gripperYawAngle;
	JointMap m_jointPositions;
	GroupMap m_jointGroups;
	
	/*
	 * This value is only valid for joint groups with the interpolation space set to CARTESIAN.
	 * 
	 * If set to true, this keyframe is part of a continuous motion. Therefore, the target 
	 * velocity is not set to zero but instead calculated to create a smooth movement.
	 */
	bool m_continuous;
};

YAML::Emitter& operator<<(YAML::Emitter& out, const KeyFrame& kf);
void operator>>(const YAML::Node& in, KeyFrame& kf);

}

#endif
