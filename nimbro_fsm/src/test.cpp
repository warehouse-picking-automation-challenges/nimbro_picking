// Test FSM

#include <nimbro_fsm/fsm_ros.h>

#include <ros/console.h>

class Driver : public nimbro_fsm::Driver
{
public:
	Driver();
	void sayHello();
	void execute();
private:
	nimbro_fsm::StateMachineROS m_fsm;
};

namespace MyTestFSM
{
	typedef nimbro_fsm::StateROS<Driver> State;

	class StateA : public State
	{
	public:
		virtual void enter()
		{
			State::enter();
		}
		virtual StateBase* execute()
		{
			ROS_INFO("execute 1, elapsed: %f", elapsedTime().toSec());
			return this;
		}
		virtual void exit()
		{
			State::exit();
			fprintf(stderr, "exit 1\n");
			ROS_INFO("exit 1");
		}
	};

	class StateB : public State
	{
	public:
		virtual void enter()
		{
			State::enter();
			ROS_INFO("enter b");
		}
		virtual State* execute()
		{
			ROS_INFO("execute b");
			driver()->sayHello();
			return this;
		}
		virtual void exit()
		{
			State::exit();
			ROS_INFO("exit b");
		}
	};

	class StateC : public State
	{
	public:
		virtual void enter()
		{
			State::enter();
			ROS_INFO("enter c");
		}
		virtual State* execute()
		{
			ROS_INFO("execute c");
			return new StateA();
		}
		virtual void exit()
		{
			State::exit();
			ROS_INFO("exit c");
		}
	};
}

Driver::Driver()
 : m_fsm(ros::NodeHandle("~"), this)
{
	m_fsm.registerState<MyTestFSM::StateB>();
	m_fsm.registerState<MyTestFSM::StateC>();

	m_fsm.changeState(new MyTestFSM::StateA());
}

void Driver::sayHello()
{
	ROS_INFO("Hello from driver");
}

void Driver::execute()
{
	m_fsm.execute();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle nh("~");

	Driver driver;

	ros::WallRate rate(ros::Duration(1.0));
	while(ros::ok())
	{
		driver.execute();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
