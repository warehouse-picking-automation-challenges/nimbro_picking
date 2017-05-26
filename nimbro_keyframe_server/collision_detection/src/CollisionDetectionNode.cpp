
#include <nimbro_collision_detection/CollisionDetectionNode.h>
#include <nimbro_collision_detection/CollisionPairs.h>
#include <std_msgs/Float32.h>
#include <moveit/robot_state/conversions.h>

#define MIN_TIMER_DURATION      0.008
#define DEFAULT_TIMER_DURATION  0.010 // Must be in the range specified by MIN/MAX_TIMER_DURATION
#define MAX_TIMER_DURATION      0.125 // Note that this isn't a hard limit

namespace collision_detection
{

CollisionDetectionNode::CollisionDetectionNode(ros::NodeHandle &n)
: m_duration("/robotcontrol/timerDuration", MIN_TIMER_DURATION, 0.0001, MAX_TIMER_DURATION, DEFAULT_TIMER_DURATION)
, m_padding("padding", 0.01, 0.001, 0.1, 0.02)
, m_serviceCalled(false)
, m_topicReceived(false)
{
	m_loader.reset(new robot_model_loader::RobotModelLoader("robot_description"));
	m_model = m_loader->getModel();
	
	m_state.reset(new robot_state::RobotState(m_model));
	m_state->setToDefaultValues();
	m_state->updateLinkTransforms();
	
	m_planningScene.reset(new planning_scene::PlanningScene(m_model));
	 	
	m_stateSub = n.subscribe("/joint_states", 1 ,&CollisionDetectionNode::stateCb, this);
	m_srv_collisionDetection = n.advertiseService("collisionDetection", &CollisionDetectionNode::srvCollisionDetection, this);
	
 	m_pub_collisionPairs = n.advertise<nimbro_collision_detection::CollisionPairs>( "collision_pairs", 0 );
	m_pub_distance = n.advertise<std_msgs::Float32>( "distance", 0 );
	
	ROS_INFO_STREAM("CollisionDetectionNode running with a padding of "<<m_padding()<<" m.");
}

bool CollisionDetectionNode::srvCollisionDetection(nimbro_collision_detection::CollisionDetectionRequest& req, nimbro_collision_detection::CollisionDetectionResponse& resp)
{	
	m_serviceCalled = true;
	if(m_serviceCalled && m_topicReceived)
	{
			ROS_WARN_STREAM("This collision detection node was called via service call and also received updates from the /joint_states topic. It is probably misconfigured.");
	}
	
	moveit::core::jointStateToRobotState(req.state, *m_state);
	
	testSelfCollision(resp);
	
	return true;
}

void CollisionDetectionNode::stateCb(const sensor_msgs::JointState& msg)
{	
	m_topicReceived = true;
	if(m_serviceCalled && m_topicReceived)
	{
			ROS_WARN_STREAM("This collision detection node was called via service call and also received updates from the /joint_states topic. It is probably misconfigured.");
	}
	
	moveit::core::jointStateToRobotState(msg, *m_state);
	
	nimbro_collision_detection::CollisionDetectionResponse empty_dummy;
	testSelfCollision(empty_dummy);
}

/**
 * This method perfors the self collision test and stores the result in resp. Furthermore, the results are also published.
 */
void CollisionDetectionNode::testSelfCollision(nimbro_collision_detection::CollisionDetectionResponse& resp)
{
	ros::Time begin = ros::Time::now();

	m_planningScene->setCurrentState(*m_state);
	
	collision_detection::CollisionRequest collision_request;
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;
	collision_request.distance = true;
	collision_detection::CollisionResult collision_result;
	
	collision_detection::CollisionRobotPtr collisionRobot  = m_planningScene->getCollisionRobotNonConst();
	collisionRobot->setPadding(m_padding());
	
 	collisionRobot->checkSelfCollision(collision_request, collision_result, m_planningScene->getCurrentStateNonConst(), m_planningScene->getAllowedCollisionMatrix());

	ros::Duration elapsedTime = ros::Time::now() - begin;

	// check if the collision checking is faster than the update rate of the joint states
	if(elapsedTime.toSec() > m_duration())
	{
		ROS_WARN_STREAM("Cannot check for collisions fast enough! Elapsed time: "<<(elapsedTime.toSec()*1000)<<" ms.");
	}
	
	// publish pairs of links which are in collision
	collision_detection::CollisionResult::ContactMap contactMap = collision_result.contacts;
	nimbro_collision_detection::CollisionPairs msg;
	nimbro_collision_detection::CollisionPair pair;
	collision_detection::CollisionResult::ContactMap::const_iterator it;
	for(it = collision_result.contacts.begin();
		it != collision_result.contacts.end();
		++it)
	{	
		std::string firstLink = it->first.first.c_str();
		std::string secondLink = it->first.second.c_str();
		
		pair.first = firstLink;
		pair.second = secondLink;
		
		msg.pairs.push_back(pair);
		resp.pairs.push_back(pair);
	}
	m_pub_collisionPairs.publish(msg);

	// publish the minimal distance between pairs
	std_msgs::Float32 distanceMsg;
	distanceMsg.data = collision_result.distance;
	m_pub_distance.publish(distanceMsg);
	resp.distance = collision_result.distance;
}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "collision_detection");
	
	ros::NodeHandle n("~"); 	
	collision_detection::CollisionDetectionNode collision_detection(n);
	
	ros::spin();
	
	return 0;
}
