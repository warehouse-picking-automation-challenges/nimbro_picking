#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <config_server/parameter.h>
#include <nimbro_collision_detection/CollisionDetection.h>

namespace collision_detection
{
	/**
	 * This node listens to the joint states and checks for collisions between links.
	 */
	class CollisionDetectionNode
	{
	public:
		CollisionDetectionNode(ros::NodeHandle& n);
		~CollisionDetectionNode(){};
	private:
		config_server::Parameter<float> m_duration;
		config_server::Parameter<float> m_padding;
		
		robot_model_loader::RobotModelLoaderPtr m_loader;
		robot_model::RobotModelPtr m_model;
		robot_state::RobotStatePtr m_state;
		planning_scene::PlanningScenePtr m_planningScene;
		
		void testSelfCollision(nimbro_collision_detection::CollisionDetectionResponse& resp);
		
 		ros::Subscriber m_stateSub;
		ros::Publisher m_pub_collisionPairs;
		ros::Publisher m_pub_distance;
		void stateCb(const sensor_msgs::JointState &msg);
		
		ros::ServiceServer m_srv_collisionDetection;
		bool srvCollisionDetection(nimbro_collision_detection::CollisionDetectionRequest& req, nimbro_collision_detection::CollisionDetectionResponse& resp);
		
		bool m_serviceCalled;
		bool m_topicReceived;
	};
}
