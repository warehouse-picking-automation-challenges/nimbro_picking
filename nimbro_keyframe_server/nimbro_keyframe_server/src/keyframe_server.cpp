// Keyframe server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <nimbro_keyframe_server/keyframe_server.h>
#include <nimbro_keyframe_server/MotionList.h>

#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <fstream>

namespace nimbro_keyframe_server
{

KeyFrameServer::KeyFrameServer()
 : m_nh("~")
{
	m_srv_getMotion = m_nh.advertiseService("getMotion", &KeyFrameServer::srvGetMotion, this);
	m_srv_setMotion = m_nh.advertiseService("setMotion", &KeyFrameServer::srvSetMotion, this);
	m_srv_play = m_nh.advertiseService("play", &KeyFrameServer::srvPlay, this);
	m_srv_reload = m_nh.advertiseService("reloadMotions", &KeyFrameServer::srvReload, this);

	m_nh.param("motion_path", m_motionPath, std::string(""));

	m_pub_motionList = m_nh.advertise<MotionList>("motionList", 1, true);

	loadMotions();
}

KeyFrameServer::~KeyFrameServer()
{
}

void KeyFrameServer::loadMotions()
{
	m_motions.clear();

	ROS_DEBUG_STREAM("Loading motions from " << m_motionPath);

	// checking if the directory exists and try to create it if it doesn't
	boost::filesystem::path motion_path(m_motionPath);
	bool directory_exists = false;
	try
	{
		if (boost::filesystem::is_directory(motion_path) ||
				boost::filesystem::create_directory(motion_path))
			directory_exists = true;
	}
	catch (const boost::filesystem::filesystem_error& e)
	{
		ROS_WARN_STREAM("KeyFrameServer: " << e.code().message() << "\n");
	}

	if (directory_exists)
	{
		boost::filesystem::directory_iterator dir(m_motionPath), end;
		BOOST_FOREACH(const boost::filesystem::path& p, std::make_pair(dir, end))
		{
			if(p.extension() != ".yaml")
				continue;

			ROS_DEBUG_STREAM("KeyFrameServer: Loading " << p);

			std::ifstream in(p.c_str());
			Motion::Ptr motion = boost::make_shared<Motion>(Motion::loadFromYAML(in));

			if(!motion->isValid())
				continue;

			m_motions[motion->name()] = motion;
		}
	}
	else
	{
		ROS_WARN_STREAM("KeyFrameServer: directory " << m_motionPath << " does not exist!\n");
	}

	publishMotionList();
}

bool KeyFrameServer::srvReload(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
	loadMotions();
	return true;
}

bool KeyFrameServer::srvGetMotion(GetMotionRequest& req, GetMotionResponse& resp)
{
	std::map<std::string, Motion::Ptr>::iterator it = m_motions.find(req.name);
	if(it == m_motions.end())
	{
		ROS_ERROR("unknown motion '%s' requested", req.name.c_str());
		return false;
	}

	Motion::Ptr motion = it->second;
	resp.motion = motion->toMsg();

	return true;
}

bool KeyFrameServer::srvSetMotion(SetMotionRequest& req, SetMotionResponse& resp)
{
	Motion::Ptr motion = boost::make_shared<Motion>(Motion::loadFromMsg(req.motion));
	resp.success = false;
	if(req.save)
	{
		bool isNewMotion = false;
		std::map<std::string, Motion::Ptr>::iterator it = m_motions.find(req.motion.name);

		if(it == m_motions.end())
		{
			isNewMotion = true;
			m_motions[req.motion.name] = motion;
		}
		else
			it->second = motion;

		std::string path = (boost::filesystem::path(m_motionPath) / (req.motion.name + ".yaml")).string();
		std::ofstream out(path.c_str());
		out << motion->serializeToYAML();

		if(out.bad())
			return false;

		if(isNewMotion)
			publishMotionList();
	}

	if(req.play && m_playCallback)
	{
		resp.success = m_playCallback(motion);
		if(!resp.success)
			return false;
	}

	return true;
}

Motion::Ptr KeyFrameServer::getMotion(std::string name)
{
	std::map<std::string, Motion::Ptr>::iterator it = m_motions.find(name);
	if (it == m_motions.end())
		return Motion::Ptr();

	return it->second;
}


bool KeyFrameServer::srvPlay(PlayRequest& req, PlayResponse& resp)
{
	std::map<std::string, Motion::Ptr>::iterator it = m_motions.find(req.name);
	if(it == m_motions.end())
		return false;

	Motion::Ptr motion = it->second;

	if(!m_playCallback)
		return false;

	return m_playCallback(motion);
}


void KeyFrameServer::setPlayCallback(const KeyFrameServer::PlayCallback& cb)
{
	m_playCallback = cb;
}

void KeyFrameServer::publishMotionList()
{
	MotionList list;
	for(std::map<std::string, Motion::Ptr>::iterator it = m_motions.begin(); it != m_motions.end(); ++it)
		list.motions.push_back(it->first);

	m_pub_motionList.publish(list);
}

}


