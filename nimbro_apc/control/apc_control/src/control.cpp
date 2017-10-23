// Control node for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include <ros/init.h>

#include <apc_control/control.h>

#include <apc_control/states/scan_pose.h>
#include <apc_control/states/shelf_regist.h>
#include <apc_control/states/flip_funnel.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <signal.h>

#include <tf_conversions/tf_eigen.h>

#include <time.h>

#include <std_srvs/Empty.h>
#include <apc_objects/apc_objects.h>
#include <std_srvs/SetBool.h>
#include <apc_interface/DimLight.h>
#include <apc_interface/SuctionStrength.h>

namespace apc_control
{

Control::Control()
  : m_nh("~"),
  m_currentItem(WorkItem()),
  m_fsm(ros::NodeHandle("~"), this),
  m_shelf(Shelf()),
  m_tf(ros::Duration(40.0)),
  m_placed_item(false),
  m_grasped_item(false)
{
	m_nh.param("json_input_path", m_jsonInputPath, std::string(""));
	m_nh.param("json_output_path", m_jsonOutputPath, std::string(""));
	m_nh.param("picking", m_picking, true);
	m_nh.param("simulation", m_simulation, false);
	m_nh.param("stow_difficulty",m_stow_difficulty, 0);


	m_sub_controllerState = m_nh.subscribe("/apc_interface/controller_state",
		1, &Control::handleControllerState, this
	);

	m_srv_skip = m_nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("skip", boost::bind(&Control::skipThisItem, this));
	m_srv_start = m_nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>("start", boost::bind(&Control::handleStart, this));


	m_pub_shelfState = m_nh.advertise<ShelfState>("shelf_state",1, true);


	if(m_simulation)
		ROS_ERROR("Simulation Mode!");


}



void Control::init()
{

	m_shelf_state.attempt_running = false;

	ROS_INFO("JsonInputPath: %s", m_jsonInputPath.c_str());
	ROS_INFO("JsonOutputPath: %s", m_jsonOutputPath.c_str());

	std::ifstream in_stream(m_jsonInputPath.c_str());

	m_shelf.loadTaskFile(in_stream);

	{
		WorkingItem w_msg;
		w_msg.name = "Initialization" ;
		w_msg.points = 0;
		w_msg.success = true;
		m_shelf_state.workingItems.push_back(w_msg);
	}

	if(m_picking)
	{
		m_shelf_state.picking = true;
		WorkOrderPick();
	}
	else
	{
		WorkingItem w_msg;
		w_msg.name = "Perception" ;
		w_msg.points = 0;
		w_msg.success = true;
		m_shelf_state.workingItems.push_back(w_msg);

		m_shelf_state.picking = false;
		WorkOrderStow();
	}


	for(auto item: m_shelf.workOrder)
	{
		WorkingItem w_msg;
		w_msg.name = item.name;
		w_msg.location = item.location;
		w_msg.destination = item.destination;
		w_msg.points = item.getItemScore();
		w_msg.success = true;
		m_shelf_state.workingItems.push_back(w_msg);
	}

	updateShelfState();

	m_robot_model_loader.reset(new robot_model_loader::RobotModelLoader(
		"robot_description", false
	));
	m_robot_model = m_robot_model_loader->getModel();

	m_robot_state.reset(new robot_state::RobotState(m_robot_model));
	m_robot_state->setToDefaultValues();

	for(auto& group : m_robot_model->getJointModelGroups())
	{
		m_robot_state->setToDefaultValues(group, "init");
	}

	computeFingerArmTransform();

	for(auto item: m_shelf.workOrder)
	{
		m_workOrderCopy.push_back(item);
	}

}

void Control::pushFailedObject(const std::string& object)
{
	m_lastFailedObjects.push_back(object);
}

std::vector<std::string> Control::lastFailedObjects() const
{
	return m_lastFailedObjects;
// 	std::vector<std::string> ret;
//
// 	for(size_t i = 0; i < std::min<size_t>(maxCount, m_lastFailedObjects.size()); ++i)
// 		ret.push_back(m_lastFailedObjects[m_lastFailedObjects.size() - 1 - i]);

// 	return ret;
}


std::vector< std::string > Control::allStowObjects()
{
	std::vector<std::string> tmp;
// 	for(auto item: m_shelf_state.workingItems)
// 	{
// 		if(item.success == true && item.name != "Initialization" && item.name != "Perception")
// 			tmp.push_back(item.name);
// 	}
// 	if(tmp.size() != 12)
// 		ROS_WARN("There are not 12 items reported!");
	for(auto item: m_workOrderCopy)
	{
		tmp.push_back(item.name);
	}
	return tmp;
}


bool Control::handleStart()
{
	m_started = true;
	return true;
}


void Control::work()
{
	m_shelf_state.attempt_running = true;
	m_shelf_state.currentWorkItem = 0;
	m_shelf_state.attemped_start_time = ros::Time::now();
	m_shelf_state.workingItems[0].start_time = ros::Time::now();
	m_pub_shelfState.publish(m_shelf_state);

	ros::WallRate rate(ros::Duration(0.1));

	bool shelf_registration;

	m_nh.param("shelf_regist", shelf_registration, true);

	if(shelf_registration)
	{
		if(m_picking)
			m_fsm.changeState(new FlipFunnel());
		else
			m_fsm.changeState(new ShelfRegist());

		while(ros::ok() && !m_shutdown && !m_fsm.finished())
		{
			m_fsm.execute();
			ros::spinOnce();
			rate.sleep();
		}
	}
	else
	{
		ROS_WARN("no shelf registration!");
	}
	// Picking Challenge
	if(m_picking)
	{
		int pickIndex;
		m_nh.param("pick_index", pickIndex, -1);

		int score = 0;
		int best_score = bestScore();

		// just one Item

		if(pickIndex >= 0)
		{
			ROS_INFO("Picking just item nr: %i ...",pickIndex);
			//TODO new item
// 			m_shelf_state.item_start_time = ros::Time::now();
// 			m_pub_shelfState.publish(m_shelf_state);

			resetItemState();

			m_currentItem = m_shelf.workOrder.at(pickIndex);

			ShelfStateNextItem(pickIndex+1);
			m_pub_shelfState.publish(m_shelf_state);

			m_fsm.changeState(new apc_control::ScanPose());

			while(ros::ok() && !m_shutdown && !m_fsm.finished())
			{
				m_fsm.execute();
				ros::spinOnce();
				rate.sleep();
			}

			// Force FSM termination
			m_fsm.changeState(0);

			ROS_INFO("FSM finished. Item '%s' was %s and %s",
				m_currentItem.name.c_str(),
				m_grasped_item ? "grasped" : "not grasped",
				m_placed_item ? "placed" : "not placed"
			);

			// Change Database if item was moved successfully.
			if(m_placed_item)
			{
				m_shelf.moveItem(m_currentItem);
				score = m_currentItem.getItemScore();
			}
			// Delete item from database (it is on the floor now)
			else if(m_grasped_item)
			{
				m_shelf.removeItem(m_currentItem);
				score = -10;
				setItemFailed();
			}
			best_score=m_currentItem.getItemScore();

// 			ShelfStateNextItem(-1);
			updateShelfState();

			ROS_INFO("Finished execution for item: %i.", pickIndex);
		}
		else
		{
			//All Items / Whole workorder
			ROS_INFO("Picking all items...");

			while(!m_shelf.workOrder.empty() && !m_shutdown)
			{
				int item_score = 0;

				//reset item state (grasped / placed)
				resetItemState();

				//take next item from queue
				m_currentItem = m_shelf.workOrder[0];

				//start item time and end time for last item
				ShelfStateNextItem(ShelfStateItemIdx(m_currentItem.name));
				m_pub_shelfState.publish(m_shelf_state);

				//delete this item from queue
				m_shelf.workOrder.erase(m_shelf.workOrder.begin());

				//start fsm
				m_fsm.changeState(new apc_control::ScanPose());
				m_arm_rotated = false;

				while(ros::ok() && !m_shutdown && !m_fsm.finished())
				{
					m_fsm.execute();
					ros::spinOnce();
					rate.sleep();
				}

				// Force FSM termination
				m_fsm.changeState(0);

				ROS_INFO("FSM finished. Item was %s and %s",
					m_grasped_item ? "grasped" : "not grasped",
					m_placed_item ? "placed" : "not placed"
				);

				// Change Database if item was moved successfully.
				if(m_placed_item)
				{
					m_shelf.moveItem(m_currentItem);
					item_score = m_currentItem.getItemScore();
				}

				// We failed at grasping or placing. Retry this item at the end ot the order.
				else
				{
					ROS_INFO("Failed to grasp Item '%s', retry at the end of queue", m_currentItem.name.c_str());

					if(m_attemped_item)
					{
						m_currentItem.attempted++;
						m_currentItem.missing = 0;
					}
					else
						m_currentItem.missing++;

					m_shelf.workOrder.push_back(m_currentItem);
					setItemFailed();
					retryItem();
				}

// 				ROS_INFO("Finished execution for item: '%s' in %f sec with a score of %i points." ,m_currentItem.name.c_str(), (ros::Time::now() - m_shelf_state.workingItems).toSec(), item_score);
				score += item_score;
				updateShelfState();
			}
			ROS_INFO("Finished execution in %f sec with a score of %i out of %i points", (ros::Time::now() - m_shelf_state.attemped_start_time).toSec(), score, best_score);

		}
	}


	//Stow Challenge
	else{
		ROS_INFO("Stowing");

		ros::Time start_order = ros::Time::now();
		int score = 0;
		int best_score = bestScore();


		//All Items / Whole workorder

		while(!m_shutdown)
		{
			//start time for percetion
			ShelfStateNextItem(1);
			m_pub_shelfState.publish(m_shelf_state);

			//reset item state (placed / grasped)
			int item_score = 0;
			resetItemState();

			//start fsm
			m_fsm.changeState(new apc_control::ScanPose());

			while(ros::ok() && !m_shutdown && !m_fsm.finished())
			{
				m_fsm.execute();
				ros::spinOnce();
				rate.sleep();
			}

			// Force FSM termination
			m_fsm.changeState(0);

			ROS_INFO("FSM finished. Item was %s and %s",
				m_grasped_item ? "grasped" : "not grasped",
				m_placed_item ? "placed" : "not placed"
			);

// 			Change Database if item was moved successfully.
			if(m_placed_item)
			{
				m_shelf.moveItem(m_currentItem);
				item_score = m_currentItem.getItemScore();
				deleteCurrentItemFromWorkOrder();

			}

// 			We failed at grasping. Retry this item.
			else
			{
				ROS_INFO("Failed to grasp Item '%s', retry again", m_currentItem.name.c_str());
				setItemFailed();
				retryItem();
				// If possible, don't try this object again

				pushFailedObject(m_currentItem.name);
			}

// 			ROS_INFO("Finished execution for item: '%s' in %f sec with a score of %i points." ,m_currentItem.name.c_str(), (ros::Time::now() - m_shelf_state.item_start_time).toSec(), item_score);
			score += item_score;
			updateShelfState();
		}

		ROS_INFO("Finished execution in %f sec with a score of %i out of %i points", (ros::Time::now() - m_shelf_state.attemped_start_time).toSec(), score, best_score);



	}

	//end timer
	ShelfStateNextItem(-1);
	m_shelf_state.attempt_running = false;
	m_pub_shelfState.publish(m_shelf_state);

	//switch off vacuum and light
	switchVacuum(false);
	switchLight(0);


	ROS_INFO("writing output...");

	for(auto& item : m_shelf.workOrder)
	{
		if(item.attempted != 0 && item.missing >= 2)
		{
			ROS_INFO("It seems that we attempted to grasp '%s' at least once, and it was not there afterwards. Placing in tote...", item.name.c_str());
			m_shelf.moveItem(item);
		}
	}

	// Get current date
	char outstr[50];
	{
		time_t t;
		struct tm *tmp;
		t = time(NULL);
		tmp = localtime(&t);

		strftime(outstr, sizeof(outstr), "%Y_%m_%d_%H_%M_%S" , tmp);
	}

	m_jsonOutputPath += "_";
	m_jsonOutputPath.append(outstr);
	m_jsonOutputPath += ".json";

	std::ofstream out_stream(m_jsonOutputPath.c_str());
	m_shelf.writeResultFile(out_stream);

	ROS_INFO("Durations per state:");
	for(auto& pair : m_stateDurations)
	{
		ROS_INFO(" - %20s: %6.2fs", pair.first.c_str(), pair.second.toSec());
	}

	ROS_INFO("Done.");
}

void Control::ShelfStateNextItem(int idx)
{
	m_shelf_state.workingItems[m_shelf_state.currentWorkItem].end_time = ros::Time::now();
	if(idx >= 0)
	{
		if(idx < (int)m_shelf_state.workingItems.size())
		{
			m_shelf_state.workingItems[idx].start_time = ros::Time::now();
			m_shelf_state.currentWorkItem = idx;
		}
		else
		{
			ROS_WARN("Item index is larger than number of items.");
		}
	}
}

int Control::ShelfStateItemIdx(std::string name)
{
	for(unsigned int i=0; i<m_shelf_state.workingItems.size(); ++i)
	{
		if(m_shelf_state.workingItems[i].name != name
			|| !m_shelf_state.workingItems[i].success
			|| m_shelf_state.workingItems[i].end_time > ros::Time(0)
		)
			continue;

		return i;
	}
	return -1;
}



void Control::updateShelfState()
{
	m_shelf_state.boxA = getBoxItems(0,0);
	m_shelf_state.boxB = getBoxItems(0,1);
	m_shelf_state.boxC = getBoxItems(0,2);
	m_shelf_state.boxD = getBoxItems(1,0);
	m_shelf_state.boxE = getBoxItems(1,1);
	m_shelf_state.boxF = getBoxItems(1,2);
	m_shelf_state.boxG = getBoxItems(2,0);
	m_shelf_state.boxH = getBoxItems(2,1);
	m_shelf_state.boxI = getBoxItems(2,2);
	m_shelf_state.boxJ = getBoxItems(3,0);
	m_shelf_state.boxK = getBoxItems(3,1);
	m_shelf_state.boxL = getBoxItems(3,2);

	m_shelf_state.tote.clear();
	for(auto item: m_shelf.tote)
	{
		m_shelf_state.tote.push_back(item.name);
	}

	m_pub_shelfState.publish(m_shelf_state);
}

void Control::setItemFailed()
{
	m_shelf_state.workingItems[m_shelf_state.currentWorkItem].success = false;
}

void Control::retryItem()
{
	WorkingItem tmp = m_shelf_state.workingItems[m_shelf_state.currentWorkItem];
	tmp.start_time = ros::Time(0);
	tmp.end_time = ros::Time(0);
	tmp.success = true;
	m_shelf_state.workingItems.push_back(tmp);
}


void Control::WorkOrderPick()
{
	m_shelf.ItemScoreDifficulty();
	std::sort(m_shelf.workOrder.begin(), m_shelf.workOrder.end(), [](const WorkItem& a, const WorkItem& b) {
		return a.objectData->difficulty < b.objectData->difficulty;
	});

}


void Control::WorkOrderStow()
{
	m_shelf.PointsAndVolume();

	std::sort(m_shelf.tote.begin(), m_shelf.tote.end(), [](const Item& a, const Item& b) {
		return a.objectData->volume() > b.objectData->volume();
	});


// 	int pointLimit = 10;
// 	switch(m_stow_difficulty)
// 	{
// 		case 0: pointLimit=10; break;
// 		case 1: pointLimit=15; break;
// 		case 2: pointLimit=20; break;
// 		default: pointLimit = 10; break;
// 	};

	std::vector<Box> boxes;
	boxes.push_back(m_shelf.boxes[3][1]);
	boxes.push_back(m_shelf.boxes[0][1]);
	boxes.push_back(m_shelf.boxes[3][0]);
	boxes.push_back(m_shelf.boxes[3][2]);
	boxes.push_back(m_shelf.boxes[0][0]);
	boxes.push_back(m_shelf.boxes[0][2]);
	boxes.push_back(m_shelf.boxes[2][1]);
	boxes.push_back(m_shelf.boxes[1][1]);
	boxes.push_back(m_shelf.boxes[2][0]);
	boxes.push_back(m_shelf.boxes[2][2]);
	boxes.push_back(m_shelf.boxes[1][0]);
	boxes.push_back(m_shelf.boxes[1][2]);

	Box* best_20p_box;
	Box* best_bigItem_box;
	bool found_best = false;
	bool found_big = false;

	for(auto box: boxes)
		ROS_WARN("box: %s p: %i vol: %f", box.name.c_str(), box.stowPoints, box.predictedVolume);

	for(auto &box: boxes)
	{
		if(box.containBigItem || box.stowPoints < 20)
			continue;

		best_20p_box = &box;
		found_best = true;
		break;
	}


	if(found_best)
		ROS_INFO("best box 20p: '%s'",best_20p_box->name.c_str());
	else
	{
		ROS_WARN("no best 20 box found! searching for 15p boxes.");
		for(auto &box: boxes)
		{
			if(box.containBigItem || box.stowPoints < 15)
				continue;

			best_20p_box = &box;
			found_best = true;
			break;
		}
		if(found_best)
			ROS_INFO("best box 15p: '%s'", best_20p_box->name.c_str());
		else
		{
			ROS_ERROR("no best box for 20 and 15p found. take box K");
			best_20p_box = &boxes[0];
		}
	}


	for(auto &box: boxes)
	{
		if(box.containBigItem || box.name == best_20p_box->name)
		{
			ROS_INFO("skip box: '%s'", box.name.c_str());
			continue;
		}
		best_bigItem_box = &box;
		found_big = true;
		box.containBigItem = true;
		break;
	}

	if(found_big)
		ROS_INFO("best box big Item: '%s' is now forbidden: %i",best_bigItem_box->name.c_str(), (int)best_bigItem_box->containBigItem);
	else
	{
		ROS_ERROR("no best big item box found! Take box B");
		best_bigItem_box = &boxes[1];
	}


	for(auto item: m_shelf.tote)
	{
		ROS_INFO("now item: '%s' big: %i", item.name.c_str(), (int)item.objectData->big);

		if(item.objectData->big)
		{
			ROS_WARN("box: '%s' p: %i big item '%s' ",  best_bigItem_box->name.c_str(), best_bigItem_box->stowPoints, item.name.c_str());

			char box_location = 'A' + best_bigItem_box->row * 3 + best_bigItem_box->col;

			WorkItem wItem(
				std::string(item.name),
				-1,
				-1,
				best_bigItem_box->row,
				best_bigItem_box->col,
				std::string("tote"),
				std::string(1,box_location),
				item.objectData);
			wItem.shelfScore = m_shelf.boxes[best_bigItem_box->row][best_bigItem_box->col].stowPoints;

			m_shelf.workOrder.push_back(wItem);

			for(auto& box: boxes)
			{
				if(box.containBigItem || box.name == best_20p_box->name)
				{
					ROS_INFO("skip box: '%s'", box.name.c_str());
					continue;
				}
				best_bigItem_box = &box;
				box.containBigItem=true;
				break;
			}

			continue;
		}

		ROS_WARN("box: '%s' p: %i item '%s' ", best_20p_box->name.c_str(), best_20p_box->stowPoints, item.name.c_str());

		char box_location = 'A' + best_20p_box->row * 3 + best_20p_box->col;

		WorkItem wItem(
			std::string(item.name),
			-1,
			-1,
			best_20p_box->row,
			best_20p_box->col,
			std::string("tote"),
			std::string(1,box_location),
			item.objectData);
		wItem.shelfScore = m_shelf.boxes[best_20p_box->row][best_20p_box->col].stowPoints;

		m_shelf.workOrder.push_back(wItem);


	}
}

void Control::switchVacuum(bool mode)
{
	if(!mode)
		setSuctionStrength(0);

	ROS_INFO("Turn vacuum %i", (int)mode);
	{
		std_srvs::SetBool srv;
		srv.request.data = mode;
		if(!ros::service::call("/apc_interface/switch_vacuum", srv))
		{
			ROS_ERROR("Could not switch vacuum %s, continuing anyway...", mode?"on":"off");
		}
	}
	ros::Duration(1.0).sleep();

	//switch off vacuum power
	if(!mode)
	{
		switchVacuumPower(false);
		setPayload(1.0f);
	}

}

void Control::switchVacuumPower(bool mode)
{
	ROS_INFO("Turn vacuum power %i", (int)mode);
	std_srvs::SetBool srv;
	srv.request.data = mode;
	if(!ros::service::call("/apc_interface/switch_vacuum_power", srv))
	{
		ROS_ERROR("Could not switch off vacuum power, continuing anyway...");
	}
}


void Control::setSuctionStrength(float value)
{
	ROS_INFO("Set suction strength to: %f", value);
	{
		apc_interface::SuctionStrength srv;
		srv.request.strength = value;
		if(!ros::service::call("/apc_interface/set_suction_strength", srv))
		{
			ROS_ERROR("Could not set suction strength to %f, continuing anyway...", value);
		}
	}
	ros::Duration(1.0).sleep();
}



void Control::switchLight(int duty)
{
	{
		apc_interface::DimLight srv;
		srv.request.duty = duty;
		if(!ros::service::call("/apc_interface/dim", srv))
		{
			ROS_ERROR("Could not switch light on, continuing anyway...");
		}
	}
}

void Control::enableServoCommunication(bool on)
{
	ROS_INFO("%s servo communication",
		(on) ? "Enable" : "Disable");
	std_srvs::SetBool srv;
	srv.request.data = on;
	if(!ros::service::call("/apc_interface/enable_servo_comm", srv))
	{
		ROS_ERROR("Could not %s communication",
			(on) ? "enable" : "disable");
	}

}

bool Control::somethingOnTip() const
{
	if(!m_controllerState)
		return false;

	if(m_currentItem.objectData->airVelThreshold == 0)
		return m_controllerState->something_on_tip;

	if(!m_currentItem.objectData)
	{
		ROS_ERROR("Current item does not have objectData!");
		return false;
	}

	ROS_INFO_THROTTLE(1.0, "Checking against threshold %f (current value: %f)", m_currentItem.objectData->airVelThreshold, m_controllerState->air_velocity_low_pass);
	return m_controllerState->air_velocity_low_pass < m_currentItem.objectData->airVelThreshold;
}


bool Control::skipThisItem()
{
	ROS_WARN("Skipping current item...");
	m_fsm.changeState(0);

	return true;
}

bool Control::setWorkItem(std::string itemName)
{
	bool success = false;
	for(auto item: m_shelf.workOrder)
	{
		if(item.name == itemName)
		{
			m_currentItem = item;
			success = true;
			break;
		}
	}

	if(!success)
	{
		for( auto item: m_workOrderCopy)
		{
			if(item.name == itemName)
			{
				m_currentItem = item;
				success = true;
				break;
			}
		}
		ROS_WARN("Reuse item '%s'.", itemName.c_str());
	}

	//Update shelf state for gui

	int idx = ShelfStateItemIdx(itemName);

	if(idx >= 0 && idx < (int)m_shelf_state.workingItems.size())
	{
		m_shelf_state.workingItems[idx].start_time = m_shelf_state.workingItems[1].start_time;
		m_shelf_state.currentWorkItem = idx;
		m_shelf_state.workingItems[1].start_time = ros::Time(0);
		m_pub_shelfState.publish(m_shelf_state);
	}
	else
	{
		ROS_WARN("Item index is out of bounds: %i",idx);
	}


	if(!success)
		ROS_ERROR("Can't find Item %s in Work Order!",itemName.c_str());
	return success;
}

float Control::getBoxHeight(int row)
{
	float box_height = apc_shelf_model::getBoxMaxZ(row);

	box_height -=0.065; //Finger hight for top grasp
	box_height -=0.010; //safety margin
	return box_height;
}


void Control::computeFingerArmTransform()
{

	m_robot_state->setVariablePosition("suc_fingertip_joint", 0);
	m_robot_state->setVariablePosition("suc_finger_joint", 0.25);

	m_robot_state->updateLinkTransforms();

	Eigen::Affine3d suc_fingertip_transform = m_robot_state->getGlobalLinkTransform("suc_fingertip_link");
	Eigen::Affine3d arm_transform = m_robot_state->getGlobalLinkTransform("apc_eef_link");

	m_finger25arm_transform = arm_transform.inverse() * suc_fingertip_transform;

	m_robot_state->setVariablePosition("suc_finger_joint",0.05);
	m_robot_state->updateLinkTransforms();
	suc_fingertip_transform = m_robot_state->getGlobalLinkTransform("suc_fingertip_link");

	m_finger05arm_transform = arm_transform.inverse() * suc_fingertip_transform;
}


std::string Control::printWorkItem(WorkItem item)
{
	std::string msg="work item: " + item.name + "\n";
	msg += item.location + "  ->  " + item.destination + "\n";
	msg += "bonus: " + std::to_string(item.objectData->bonusPoints) + "p\n";
	msg += "shelf score: " + std::to_string(item.shelfScore) + "p\n";
	msg += "difficulty: " + std::to_string(item.objectData->difficulty) + "\n";

	return msg;
}


void Control::handleControllerState(const apc_interface::ControllerStateConstPtr& msg)
{
	m_controllerState = msg;
}

Eigen::Affine3d Control::fingerPose() const
{
	Eigen::Affine3d ret;

	tf::StampedTransform tfTrans;
	try
	{
		m_tf.lookupTransform("world", "suc_cup_base_link", ros::Time(0), tfTrans);
	}
	catch(const tf::TransformException& e)
	{
		ROS_ERROR("Could not get finger tip pose! Returning identity...");
		tfTrans.setIdentity();
	}

	tf::transformTFToEigen(tfTrans, ret);
	return ret;
}

std::string Control::report_error(int error_code)
{
	std::string msg;
	switch(error_code)
	{
		case PlayResult::TIMEOUT: msg="TIMEOUT"; break;
		case PlayResult::COLLISION: msg="COLLISION"; break;
		case PlayResult::EMERGENCYSTOP: msg="EMERGENCYSTOP"; break;
		case PlayResult::PREEMPTED: msg="PREEMPTED"; break;
		case PlayResult::INVALID_GOAL: msg="INVALID_GOAL"; break;
		case PlayResult::INVALID_JOINTS: msg="INVALID_JOINTS"; break;
		default: msg="invalid error message";
	}

	return msg;
}

void Control::resetItemState()
{
	m_placed_item = false;
	m_grasped_item = false;
	m_attemped_item = false;
}

int Control::bestScore()
{
	int score=0;
	if(m_picking)
	{
		for(auto item: m_shelf.workOrder)
		{
			score += item.getItemScore();
		}
		return score;
	}
	else
	{
		score = 12 * 20;
		for(auto item: m_shelf.workOrder)
		{
			score += item.objectData->bonusPoints;
		}
	}
	return score;
}

void Control::deleteCurrentItemFromWorkOrder()
{
	for(unsigned int i=0;i<m_shelf.workOrder.size(); i++)
	{
		if(m_shelf.workOrder[i].name == m_currentItem.name)
		{
			m_shelf.workOrder.erase(m_shelf.workOrder.begin() + i);
			return;
		}
	}
	ROS_ERROR("Can't find Item '%s' in the work order!", m_currentItem.name.c_str());
}


std::vector<std::string> Control::getBoxItems(int row, int col)
{
	std::vector<std::string> items;
	for(unsigned int i=0;i<m_shelf.boxes[row][col].items.size();i++)
	{
		items.push_back(m_shelf.boxes[row][col].items[i].name);
	}
	return items;
}



void Control::shutdown()
{
	fprintf(stderr, "Shutdown requested\n");
	m_shutdown = true;
	m_shelf_state.attempt_running = false;
}

void Control::addStateDuration(const std::string& state, const ros::Duration& duration)
{
	m_stateDurations[state] += duration;
}

void Control::setPayload(float payload)
{
// 	ur_msgs::SetPayload srv;
//
// 	srv.request.payload = payload;
//
// 	if(!ros::service::call("/ur_driver/set_payload", srv))
// 	{
// 		ROS_ERROR("Could not call /ur_driver/set_payload");
// 	}
}

}

static std::unique_ptr<apc_control::Control> g_control;

void handleShutdown(int)
{
	g_control->shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "controller", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	// Since actionlib is stupid, we need to cancel the active goals
	// *before* ros::shutdown() is called (or more precisely, before
	// ros::ok() returns false).
	signal(SIGINT, &handleShutdown);

	g_control.reset(new apc_control::Control);

	g_control->init();

	ros::WallRate rate(ros::Duration(0.1));

	while(ros::ok())
	{
		if(g_control->m_started)
			break;

		ros::spinOnce();
		rate.sleep();
	}
	if(ros::ok())
		g_control->work();

	usleep(500 * 1000);

	return 0;
}
