// Represents a single motion
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NIMBRO_KEYFRAME_SERVER_MOTION_H
#define NIMBRO_KEYFRAME_SERVER_MOTION_H

#include <vector>

#include <nimbro_keyframe_server/keyframe.h>

#include <boost/shared_ptr.hpp>

#include <nimbro_keyframe_server/MotionMsg.h>

namespace nimbro_keyframe_server
{

class Motion : public std::vector<KeyFrame>
{
public:
	typedef boost::shared_ptr<Motion> Ptr;

	explicit Motion(const std::string& name);
	Motion();
	~Motion();

	bool isValid() const;

	inline const std::string& name() const
	{ return m_name; }

	void setName(std::string name)
	{m_name = name; }


	std::string serializeToYAML() const;

	MotionMsg toMsg() const;

	KeyFrame* findKeyframe(std::string label);

	static Motion loadFromYAML(std::istream& stream);
	static Motion loadFromMsg(const MotionMsg& msg);


	Eigen::Affine3d referencePose;
	std::string referenceObjectMesh;

private:

	std::string m_name;
	int m_version;
};

typedef boost::shared_ptr<Motion> MotionPtr;
}

#endif
