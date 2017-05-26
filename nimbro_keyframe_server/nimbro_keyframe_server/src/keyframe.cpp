// A single keyframe in endeffector space
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_keyframe_server/keyframe.h>

#include <yaml-cpp/yaml.h>

#include <eigen_conversions/eigen_msg.h>

#include <ros/console.h>

namespace nimbro_keyframe_server
{

KeyFrame::JointGroup::JointGroup()
 : m_interpolation_space(KeyFrame::IS_NONE)
 , m_state(Eigen::Affine3d::Identity())
{
}

KeyFrame::JointGroup::JointGroup(InterpolationSpace interpolation_space)
: m_interpolation_space(interpolation_space)
, m_state(Eigen::Affine3d::Identity())
{
}

KeyFrame::JointGroup::JointGroup(InterpolationSpace space, const Eigen::Affine3d& state)
 : m_interpolation_space(space)
 , m_state(state)
{
}

void KeyFrame::JointGroup::setInterpolationSpace(InterpolationSpace space)
{
	m_interpolation_space = space;
}

void KeyFrame::JointGroup::setState(const Eigen::Affine3d& state)
{
	m_state = state;
}

KeyFrame::KeyFrame()
 : m_linearVelocity(0) //for historical reasons
 , m_angularVelocity(0)
 , m_jointSpaceVelocity(NAN)
 , m_linearAcceleration(NAN)
 , m_angularAcceleration(NAN)
 , m_jointSpaceAcceleration(NAN)
 , m_torqueProportion(-1.0)
 , m_gripperAngle(0)
 , m_gripperYawAngle(0)
 , m_jointGroups()
 , m_continuous(false)
{
}

void KeyFrame::addJointGroup(std::string groupName, JointGroup group)
{
	m_jointGroups[groupName] = group;
}

void KeyFrame::setJointPosition(std::string name, double position)
{
	m_jointPositions[name] = position;
}

void KeyFrame::removeJointPositions(std::vector<std::string>& jointNames)
{
	for(unsigned int i = 0; i < jointNames.size(); ++i)
	{
		m_jointPositions.erase(jointNames[i]);
	}		
}

void KeyFrame::setLabel(const std::string& label)
{
	m_label = label;
}

void KeyFrame::setAngularVelocity(double vel)
{
	m_angularVelocity = vel;
}

void KeyFrame::setTorqueProportion(double torqueProportion)
{
	m_torqueProportion = torqueProportion;
}

void KeyFrame::setLinearVelocity(double vel)
{
	m_linearVelocity = vel;
	m_jointSpaceVelocity = vel;
}

void KeyFrame::setInterpolationSpace(std::string groupName, InterpolationSpace space)
{
	m_jointGroups[groupName].setInterpolationSpace(space);
}

bool KeyFrame::hasJointGroup(std::string groupName) const
{
	if (m_jointGroups.find(groupName) != m_jointGroups.end())
		return true;
	return false;
}

void KeyFrame::setState(std::string groupName, const Eigen::Affine3d& state)
{
	m_jointGroups[groupName].setState(state);
}


YAML::Emitter& operator<<(YAML::Emitter& out, const KeyFrame& kf)
{

	out << YAML::BeginMap;

	out << YAML::Key << "active_joint_groups";
	out << YAML::Value;

	const KeyFrame::GroupMap& groups = kf.jointGroups();

	out << YAML::BeginMap;
	KeyFrame::GroupMap::const_iterator group = groups.begin();
	for (; group != groups.end(); ++group)
	{
		out << YAML::Key << group->first;
		out << YAML::Value;

		out << YAML::BeginMap;

		out << YAML::Key << "interpolation_space";
		switch(group->second.interpolationSpace())
		{
			case KeyFrame::IS_CARTESIAN:
				out << YAML::Value << "cartesian";
				break;
			case KeyFrame::IS_JOINT_SPACE:
				out << YAML::Value << "jointspace";
				break;
			case KeyFrame::IS_NONE:
				out << YAML::Value << "none";
				break;
			default:
				abort();
		}


		if (group->second.interpolationSpace() == KeyFrame::IS_CARTESIAN)
		{
			out << YAML::Key << "state";
			out << YAML::Value;

			const Eigen::Affine3d& state = group->second.state();
			out << YAML::BeginMap;
			out << YAML::Key << "x";
			out << YAML::Value << state.translation().x();
			out << YAML::Key << "y";
			out << YAML::Value << state.translation().y();
			out << YAML::Key << "z";
			out << YAML::Value << state.translation().z();

			Eigen::Matrix3d rot = state.rotation();
			Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);
			out << YAML::Key << "roll";
			out << YAML::Value << euler(2);
			out << YAML::Key << "pitch";
			out << YAML::Value << euler(1);
			out << YAML::Key << "yaw";
			out << YAML::Value << euler(0);
			out << YAML::EndMap;
		}
		out << YAML::EndMap;
	}
	out << YAML::EndMap;


	out << YAML::Key << "joints" << YAML::Value;
	out << YAML::BeginMap;
	KeyFrame::JointMap::const_iterator joint = kf.jointPositions().begin();
	for (;joint != kf.jointPositions().end(); ++joint)
	{
		out << YAML::Key << joint->first;
		out << YAML::Value << joint->second;
	}
	out << YAML::EndMap;

	out << YAML::Key << "label";
	out << YAML::Value << kf.label();

	if(std::isfinite(kf.linearVelocity()))
	{
		out << YAML::Key << "linearVelocity";
		out << YAML::Value << kf.linearVelocity();
	}

	if(std::isfinite(kf.angularVelocity()))
	{
		out << YAML::Key << "angularVelocity";
		out << YAML::Value << kf.angularVelocity();
	}

	if(std::isfinite(kf.jointSpaceVelocity()))
	{
		out << YAML::Key << "jointSpaceVelocity";
		out << YAML::Value << kf.jointSpaceVelocity();
	}

	if(std::isfinite(kf.linearAcceleration()))
	{
		out << YAML::Key << "linearAcceleration";
		out << YAML::Value << kf.linearAcceleration();
	}

	if(std::isfinite(kf.angularAcceleration()))
	{
		out << YAML::Key << "angularAcceleration";
		out << YAML::Value << kf.angularAcceleration();
	}

	if(std::isfinite(kf.jointSpaceAcceleration()))
	{
		out << YAML::Key << "jointSpaceAcceleration";
		out << YAML::Value << kf.jointSpaceAcceleration();
	}

	out << YAML::Key << "torqueProportion";
	out << YAML::Value << kf.torqueProportion();

	out << YAML::Key << "continuous";
	out << YAML::Value << kf.continuous();

	out << YAML::EndMap;

	return out;
}

void KeyFrame::loadYAML (const YAML::Node& in, int vers)
{

	const YAML::Node& joint_groups = in["active_joint_groups"];
	for(YAML::const_iterator group = joint_groups.begin(); group != joint_groups.end(); ++group)
	{
		std::string groupName = group->first.as<std::string>();
		KeyFrame::JointGroup jointGroup;

		std::string int_space = group->second["interpolation_space"].as<std::string>();
		if (int_space == "linear" or int_space == "cartesian")
			jointGroup.setInterpolationSpace(KeyFrame::IS_CARTESIAN);
		else if (int_space  == "joint_space" or int_space == "jointspace")
			jointGroup.setInterpolationSpace(KeyFrame::IS_JOINT_SPACE);
		else if (int_space == "none")
			jointGroup.setInterpolationSpace(KeyFrame::IS_NONE);
		else
			ROS_WARN("Ignoring unknown interpolation space '%s'", int_space.c_str());

		if (int_space == "linear" || int_space == "cartesian")
		{
			const YAML::Node& state_yaml = group->second["state"];
			Eigen::Affine3d state = Eigen::Affine3d::Identity();
			double x,y,z,roll,pitch,yaw;
			x = state_yaml["x"].as<double>();
			y = state_yaml["y"].as<double>();
			z = state_yaml["z"].as<double>();
			roll = state_yaml["roll"].as<double>();
			pitch = state_yaml["pitch"].as<double>();
			yaw = state_yaml["yaw"].as<double>();
			state.translate(Eigen::Vector3d(x, y, z));

			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
			state.rotate(rot);

			jointGroup.setState(state);
		}
		else
			jointGroup.setState(Eigen::Affine3d::Identity());

		m_jointGroups[groupName] = jointGroup;
// 		kf.addJointGroup(groupName, jointGroup);
	}

	const YAML::Node& joint_list = in["joints"];
	for(YAML::const_iterator it=joint_list.begin();it!=joint_list.end();++it) {
		m_jointPositions[it->first.as<std::string>()] = it->second.as<double>();
// 		kf.setJointPosition(it->first.as<std::string>(), it->second.as<double>());
	}

	m_label = in["label"].as<std::string>();

	if(in["linearVelocity"])
		m_linearVelocity = in["linearVelocity"].as<double>();
	if(in["angularVelocity"])
		m_angularVelocity = in["angularVelocity"].as<double>();
	if(in["jointSpaceVelocity"])
	{
		if(vers<2)
			m_jointSpaceVelocity = in["linearVelocity"].as<double>();
		else
			m_jointSpaceVelocity = in["jointSpaceVelocity"].as<double>();
	}

	if(in["linearAcceleration"])
		m_linearAcceleration = in["linearAcceleration"].as<double>();
	if(in["angularAcceleration"])
		m_angularAcceleration = in["angularAcceleration"].as<double>();
	if(in["jointSpaceAcceleration"])
		m_jointSpaceAcceleration = in["jointSpaceAcceleration"].as<double>();


	if(in["torqueProportion"])
		m_torqueProportion = in["torqueProportion"].as<double>();
	else
		m_torqueProportion = -1.0f;
	if(in["continuous"])
		m_continuous = in["continuous"].as<bool>();
	else
		m_continuous = false;
}

KeyFrameMsg KeyFrame::toMsg() const
{
	KeyFrameMsg msg;
	msg.linearVelocity = m_linearVelocity;
	msg.angularVelocity = m_angularVelocity;
	msg.jointSpaceVelocity = m_jointSpaceVelocity;

	msg.linearAcceleration = m_linearAcceleration;
	msg.angularAcceleration = m_angularAcceleration;
	msg.jointSpaceAcceleration = m_jointSpaceAcceleration;

	msg.torqueProportion = m_torqueProportion;
	msg.continuous = m_continuous;
	msg.gripperAngle = m_gripperAngle;

	GroupMap::const_iterator group = m_jointGroups.begin();
	for (; group != m_jointGroups.end(); ++group)
	{
		if(group->second.interpolationSpace() ==  KeyFrame::IS_NONE)
			continue;
		
		JointGroupMsg groupMsg;
		groupMsg.groupName = group->first;
		groupMsg.interpolationSpace = group->second.interpolationSpace();
		tf::poseEigenToMsg(group->second.state(), groupMsg.state);
		msg.activeJointGroups.push_back(groupMsg);
	}


	std::map<std::string, double>::const_iterator j =  m_jointPositions.begin();
	for (; j != m_jointPositions.end(); ++j )
	{
		JointPosition joint;
		joint.name = j->first;
		joint.position = j->second;
		msg.joints.push_back(joint);
	}


	msg.label = m_label;

	return msg;
}

KeyFrame KeyFrame::fromMsg(const KeyFrameMsg& msg)
{
	KeyFrame kf;
	kf.setAngularVelocity(msg.angularVelocity);
	kf.setCartLinearVelocity(msg.linearVelocity);
	kf.setJointSpaceVelocity(msg.jointSpaceVelocity);

	kf.setAngularAcceleration(msg.angularAcceleration);
	kf.setLinearAcceleration(msg.linearAcceleration);
	kf.setJointSpaceAcceleration(msg.jointSpaceAcceleration);

	kf.setTorqueProportion(msg.torqueProportion);
	kf.setContinuous(msg.continuous);

	size_t i;
	for (i = 0; i < msg.joints.size(); i++)
		kf.setJointPosition(msg.joints[i].name, msg.joints[i].position);

	std::vector<JointGroupMsg>::const_iterator group_msg = msg.activeJointGroups.begin();
	for (; group_msg != msg.activeJointGroups.end(); ++group_msg)
	{
		std::string groupName = group_msg->groupName;
		Eigen::Affine3d state;
		tf::poseMsgToEigen(group_msg->state, state);
		InterpolationSpace space = (InterpolationSpace)group_msg->interpolationSpace;
		KeyFrame::JointGroup group(space, state);
		kf.addJointGroup(groupName, group);

	}

	kf.setLabel(msg.label);
	
	return kf;
}

void KeyFrame::setContinuous(const bool continuous)
{
	m_continuous = continuous;
}

}
