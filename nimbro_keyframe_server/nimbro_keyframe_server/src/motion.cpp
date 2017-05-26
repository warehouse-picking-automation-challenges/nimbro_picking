// Represents a single motion
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_keyframe_server/motion.h>

#include <yaml-cpp/yaml.h>

#include <eigen_conversions/eigen_msg.h>

#include <ros/console.h>


namespace nimbro_keyframe_server
{

void operator>>(const YAML::Node& in, Eigen::Affine3d& pose)
{
	pose = Eigen::Affine3d::Identity();
	double x,y,z,roll,pitch,yaw;
	x = in["x"].as<double>();
	y = in["y"].as<double>();
	z = in["z"].as<double>();
	roll = in["roll"].as<double>();
	pitch = in["pitch"].as<double>();
	yaw = in["yaw"].as<double>();
	pose.translate(Eigen::Vector3d(x, y, z));

	Eigen::Matrix3d rot;
	rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
	pose.rotate(rot);
}

YAML::Emitter& operator<<(YAML::Emitter& out, const Eigen::Affine3d& pose)
{
	out << YAML::BeginMap;
	out << YAML::Key << "x";
	out << YAML::Value << pose.translation().x();
	out << YAML::Key << "y";
	out << YAML::Value << pose.translation().y();
	out << YAML::Key << "z";
	out << YAML::Value << pose.translation().z();

	Eigen::Matrix3d rot = pose.rotation();
	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);
	out << YAML::Key << "roll";
	out << YAML::Value << euler(2);
	out << YAML::Key << "pitch";
	out << YAML::Value << euler(1);
	out << YAML::Key << "yaw";
	out << YAML::Value << euler(0);
	out << YAML::EndMap;

	return out;
}

bool isValidPose(const Eigen::Affine3d& pose)
{
	//Note(sebastian): Will return false if still set to NaN
	return std::isfinite(pose.translation().x());
}


Motion::Motion()
 : referencePose(Eigen::Affine3d::Identity())
 , referenceObjectMesh("")
 , m_version(2)
{
	//Note(sebastian): Use NaN as initial value to be able to see if
	// the pose is set during initialization
	referencePose.translate(Eigen::Vector3d(NAN, NAN, NAN));
}

Motion::Motion(const std::string& name)
 : referencePose(Eigen::Affine3d::Identity())
 , referenceObjectMesh("")
 , m_name(name)
 , m_version(2)
{
	//Note(sebastian): Use NaN as initial value to be able to see if
	// the pose is set during initialization
	referencePose.translate(Eigen::Vector3d(NAN, NAN, NAN));
}

Motion::~Motion()
{
}

bool Motion::isValid() const
{
	return !m_name.empty();
}

Motion Motion::loadFromYAML(std::istream& stream)
{
	YAML::Node doc = YAML::Load(stream);
	if(doc.IsNull())
	{
		ROS_WARN("KeyframeServer: Could not get YAML document");
		return Motion();
	}

	try
	{
		std::string name = doc["name"].as<std::string>();

		Motion motion(name);

		if(doc["version"])
			motion.m_version = doc["version"].as<int>();
		else
			motion.m_version = 1;

		const YAML::Node& frame_list = doc["keyframes"];
		for(size_t i = 0; i < frame_list.size(); ++i)
		{
			KeyFrame frame;
			frame.loadYAML(frame_list[i], motion.m_version);

			motion.push_back(frame);
		}

		if(doc["reference_object"])
		{
			const YAML::Node& obj = doc["reference_object"];
			const YAML::Node& pose = obj["pose"];
			pose >> motion.referencePose;
			motion.referenceObjectMesh = obj["mesh"].as<std::string>();
		}

		return motion;
	}
	catch(YAML::Exception& e)
	{
		ROS_WARN("Could not parse YAML: %s", e.what());
		return Motion();
	}
}

std::string Motion::serializeToYAML() const
{
	YAML::Emitter out;

	out << YAML::BeginMap;

	out << YAML::Key << "name";
	out << YAML::Value << m_name;

	out << YAML::Key << "version";
	out << YAML::Value << m_version;

	out << YAML::Key << "keyframes";
	out << YAML::Value << YAML::BeginSeq;
	for(size_t i = 0; i < size(); ++i)
	{
		out << at(i);
	}
	out << YAML::EndSeq;

	if(isValidPose(referencePose))
	{
		out << YAML::Key << "reference_object";
		out << YAML::Value << YAML::BeginMap;
		out << YAML::Key << "pose";
		out << YAML::Value << referencePose;
		out << YAML::Key << "mesh";
		out << YAML::Value << referenceObjectMesh;
		out << YAML::EndMap;
	}

	out << YAML::EndMap;

	return out.c_str();
}

MotionMsg Motion::toMsg() const
{
	MotionMsg msg;
	msg.name = m_name;
	msg.reference_object_mesh = referenceObjectMesh;
	tf::poseEigenToMsg(referencePose, msg.reference_pose);

	for(size_t i = 0; i < size(); ++i)
		msg.keyframes.push_back(at(i).toMsg());

	return msg;
}

KeyFrame* Motion::findKeyframe(std::string label)
{
	Motion::iterator it;
	for (it = begin(); it != end(); ++it)
	{
		if (it->label() == label)
			return &*(it);
	}
	return 0;
}

Motion Motion::loadFromMsg(const MotionMsg& msg)
{
	Motion motion(msg.name);
	motion.m_version = 2;
	for(size_t i = 0; i < msg.keyframes.size(); ++i)
		motion.push_back(KeyFrame::fromMsg(msg.keyframes[i]));

	motion.referenceObjectMesh = msg.reference_object_mesh;
	tf::poseMsgToEigen(msg.reference_pose, motion.referencePose);

	return motion;
}


}
