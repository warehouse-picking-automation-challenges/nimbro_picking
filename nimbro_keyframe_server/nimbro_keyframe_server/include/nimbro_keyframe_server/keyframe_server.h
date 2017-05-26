// Keyframe server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_KEYFRAME_SERVER_H
#define NIMBRO_KEYFRAME_SERVER_H

#include <nimbro_keyframe_server/motion.h>
#include <map>

#include <ros/node_handle.h>

#include <nimbro_keyframe_server/GetMotion.h>
#include <nimbro_keyframe_server/SetMotion.h>
#include <nimbro_keyframe_server/Play.h>

#include <std_srvs/Empty.h>

namespace nimbro_keyframe_server
{

class KeyFrameServer
{
public:
	KeyFrameServer();
	~KeyFrameServer();

	typedef boost::function<bool (const Motion::Ptr&)> PlayCallback;

	void setPlayCallback(const PlayCallback& cb);

	Motion::Ptr getMotion(std::string name);
private:
	void loadMotions();

	bool srvGetMotion(nimbro_keyframe_server::GetMotionRequest& req, nimbro_keyframe_server::GetMotionResponse& resp);
	bool srvSetMotion(nimbro_keyframe_server::SetMotionRequest& req, nimbro_keyframe_server::SetMotionResponse& resp);
	bool srvPlay(nimbro_keyframe_server::PlayRequest& req, nimbro_keyframe_server::PlayResponse& resp);
	bool srvReload(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

	void publishMotionList();

	ros::NodeHandle m_nh;
	std::map<std::string, Motion::Ptr> m_motions;

	std::string m_motionPath;

	ros::Publisher m_pub_motionList;
	ros::ServiceServer m_srv_getMotion;
	ros::ServiceServer m_srv_setMotion;
	ros::ServiceServer m_srv_play;
	ros::ServiceServer m_srv_reload;

	PlayCallback m_playCallback;
};

}

#endif
