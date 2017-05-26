// Provides markers to edit a RobotState in IK space
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "ik_view.h"

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <nimbro_keyframe_server/SetMotion.h>
#include <QItemSelectionModel>

#include <algorithm>

#include <eigen_conversions/eigen_msg.h>

#include "motion_model.h"

namespace nimbro_keyframe_editor
{

IKView::IKView(const robot_state::RobotStatePtr& state, const boost::shared_ptr<tf::Transformer>& transformer, QObject* parent)
 : QObject(parent)
 , m_transformer(transformer)
 , m_kfState(state)
 , m_robotModel(state->getRobotModel())
 , m_modelRow(-1)
 , m_markerServer("mean_marker")
{
	m_meanMarker.name = "mean";
	m_meanMarker.header.frame_id = "base_link";
	m_meanMarker.scale = 1.0;
	m_meanMarker.controls.resize(2);
	m_meanMarker.controls[0].name = "move";
	m_meanMarker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
	m_meanMarker.controls[0].always_visible = true;
	m_meanMarker.controls[1].name = "icon";
	m_meanMarker.controls[1].interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
	m_meanMarker.controls[1].markers.resize(1);
	m_meanMarker.controls[1].markers[0].id = 0;
	m_meanMarker.controls[1].markers[0].type = visualization_msgs::Marker::CUBE;
	m_meanMarker.controls[1].markers[0].color.r = 1.0;
	m_meanMarker.controls[1].markers[0].color.g = 0.0;
	m_meanMarker.controls[1].markers[0].color.b = 0.0;
	m_meanMarker.controls[1].markers[0].color.a = 1.0;
	m_meanMarker.controls[1].markers[0].scale.x = 0.2;
	m_meanMarker.controls[1].markers[0].scale.y = 0.2;
	m_meanMarker.controls[1].markers[0].scale.z = 0.2;

	{
		visualization_msgs::InteractiveMarkerControl control;
		control.orientation.w = 1;
		control.orientation.x = 1;
		control.orientation.y = 0;
		control.orientation.z = 0;
		control.name = "rotate_x";
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		m_meanMarker.controls.push_back(control);
		control.name = "move_x";
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		m_meanMarker.controls.push_back(control);

		control.orientation.w = 1;
		control.orientation.x = 0;
		control.orientation.y = 1;
		control.orientation.z = 0;
		control.name = "rotate_z";
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		m_meanMarker.controls.push_back(control);
		control.name = "move_z";
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		m_meanMarker.controls.push_back(control);

		control.orientation.w = 1;
		control.orientation.x = 0;
		control.orientation.y = 0;
		control.orientation.z = 1;
		control.name = "rotate_y";
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		m_meanMarker.controls.push_back(control);
		control.name = "move_y";
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		m_meanMarker.controls.push_back(control);
	}

	m_markerServer.insert(m_meanMarker, boost::bind(&IKView::handleMeanUpdate, this, _1));
	m_markerServer.applyChanges();

	for(auto jointGroup : m_robotModel->getJointModelGroups())
	{
		if(!jointGroup->getSolverInstance())
			continue;

		GroupHandler groupHandler;
		groupHandler.group = jointGroup;
		groupHandler.interaction.reset(
			new robot_interaction::RobotInteraction(m_robotModel, jointGroup->getName())
		);
		groupHandler.interaction->decideActiveComponents(jointGroup->getName());

		groupHandler.handler.reset(
			new robot_interaction::InteractionHandler(jointGroup->getName(), *state, m_transformer)
		);
		groupHandler.interaction->addInteractiveMarkers(groupHandler.handler);
		groupHandler.handler->setUpdateCallback(boost::bind(&IKView::triggerInteractiveUpdate, this, m_handlers.size()));

		m_handlers.push_back(groupHandler);

		ROS_INFO("InteractiveKeyFrameView: group '%s' ready", jointGroup->getName().c_str());
	}

	// Helper connection to execute interactiveUpdate() in own thread
	connect(this, SIGNAL(triggerInteractiveUpdate(int)), SLOT(handleInteractiveUpdate(int)), Qt::QueuedConnection);
	m_disabledKeyframes = false;
}

IKView::~IKView()
{
}

void IKView::setModel(MotionModel* model)
{
	m_model = model;

	connect(m_model, SIGNAL(rowsInserted(QModelIndex, int, int)), SLOT(insertRows(QModelIndex,int,int)));
	connect(m_model, SIGNAL(rowsRemoved(QModelIndex, int, int)), SLOT(removeRows(QModelIndex,int,int)));
	connect(m_model, SIGNAL(modelReset()), SLOT(modelReset()));
}

void IKView::setSelectionModel(QItemSelectionModel* selectionModel)
{
	connect(selectionModel, SIGNAL(selectionChanged(QItemSelection,QItemSelection)), SLOT(handleSelection(QItemSelection)));
	m_selectionModel = selectionModel;
}

void IKView::updateActiveGroups()
{
	for(auto group : m_handlers)
		group.interaction->clearInteractiveMarkers();

	if(m_modelRow != -1)
	{
		const auto& kf = m_model->motion()[m_modelRow];

		for(auto groupHandler : m_handlers)
		{
			// Skip any non-IK groups
			const auto& kfGroups = kf.jointGroups();
			auto it = kfGroups.find(groupHandler.group->getName());

			if(it == kfGroups.end() || it->second.interpolationSpace() != nimbro_keyframe_server::KeyFrame::IS_CARTESIAN)
				continue;

			groupHandler.interaction->addInteractiveMarkers(groupHandler.handler);
		}
	}

	for(auto group : m_handlers)
		group.interaction->publishInteractiveMarkers();

	if(m_modelRow != -1)
	{
		updateFromModel();
	}
}

void IKView::handleSelection(const QItemSelection& selection)
{
	if(selection.count() != 0)
		m_modelRow = selection[0].topLeft().row();
	else
		m_modelRow = -1;

	updateActiveGroups();
}

void IKView::modelReset()
{
	for(auto group : m_handlers)
	{
		group.interaction->clearInteractiveMarkers();
		group.interaction->publishInteractiveMarkers();
	}

	m_modelRow = -1;
}

void IKView::updateFromModel()
{
	m_kfState->updateLinkTransforms();

	for(auto groupHandler : m_handlers)
	{
		groupHandler.handler->setState(*m_kfState);
		groupHandler.interaction->updateInteractiveMarkers(groupHandler.handler);
	}

	updateMeanMarker();
}

void IKView::handleInteractiveUpdate(int index)
{
	GroupHandler* groupHandler = &m_handlers[index];

	auto state = groupHandler->handler->getState();

	const std::vector<robot_interaction::RobotInteraction::EndEffector>& endEffectors = groupHandler->interaction->getActiveEndEffectors();
	if(endEffectors.size() != 0)
	{
		const robot_interaction::RobotInteraction::EndEffector& eef = endEffectors[0];
		if(!groupHandler->handler->inError(eef))
		{
			std::vector<double> positions(groupHandler->group->getVariableCount());
			state->copyJointGroupPositions(groupHandler->group, positions);

			m_kfState->setJointGroupPositions(groupHandler->group, positions);

			changed();
			updateMeanMarker();
		}
	}
}

void IKView::updateFromState()
{
	updateFromModel();
}
void IKView::disableKeyframes(bool disable)
{
	m_disabledKeyframes = disable;
}

void IKView::updateMeanMarker()
{
	if(m_modelRow == -1)
	{
		return;
	}

	const auto& kf = m_model->motion()[m_modelRow];
	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	unsigned int count = 0;

	for(auto& groupHandler : m_handlers)
	{
		// Skip any non-IK groups
		const auto& kfGroups = kf.jointGroups();
		auto it = kfGroups.find(groupHandler.group->getName());

		if(it == kfGroups.end() || it->second.interpolationSpace() != nimbro_keyframe_server::KeyFrame::IS_CARTESIAN)
			continue;

		auto state = groupHandler.handler->getState();
		auto transform = state->getGlobalLinkTransform(groupHandler.interaction->getActiveEndEffectors()[0].parent_link);

		mean += transform.translation();
		count++;
	}

	if(count == 0)
		return;

	mean /= count;

	Eigen::Affine3d trans;
	trans = Eigen::Translation3d(mean);

	tf::poseEigenToMsg(trans, m_meanMarker.pose);

	m_markerServer.insert(m_meanMarker);
	m_markerServer.applyChanges();
}

void IKView::handleMeanUpdate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb)
{
	if(m_modelRow == -1)
	{
		return;
	}

	const auto& kf = m_model->motion()[m_modelRow];

	if(fb->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN)
	{
		ROS_INFO("MOUSE DOWN");

		for(auto& groupHandler : m_handlers)
		{
			// Skip any non-IK groups
			const auto& kfGroups = kf.jointGroups();
			auto it = kfGroups.find(groupHandler.group->getName());

			if(it == kfGroups.end() || it->second.interpolationSpace() != nimbro_keyframe_server::KeyFrame::IS_CARTESIAN)
				continue;

			auto state = groupHandler.handler->getState();

			auto transform = state->getGlobalLinkTransform(groupHandler.interaction->getActiveEndEffectors()[0].parent_link);
			groupHandler.meanStartPose = transform;
		}

		tf::poseMsgToEigen(fb->pose, m_meanMarkerStart);
	}

	Eigen::Affine3d start = m_meanMarkerStart;

	Eigen::Affine3d end;
	tf::poseMsgToEigen(fb->pose, end);

	Eigen::Affine3d diff = start.inverse() * end;

	for(auto& groupHandler : m_handlers)
	{
		// Skip any non-IK groups
		const auto& kfGroups = kf.jointGroups();
		auto it = kfGroups.find(groupHandler.group->getName());

		if(it == kfGroups.end() || it->second.interpolationSpace() != nimbro_keyframe_server::KeyFrame::IS_CARTESIAN)
			continue;

		m_kfState->setFromIK(groupHandler.group, diff * groupHandler.meanStartPose);
		m_kfState->updateLinkTransforms();

		groupHandler.handler->setState(*m_kfState);
		groupHandler.interaction->updateInteractiveMarkers(groupHandler.handler);
	}

	changed();

	if(fb->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
	{
		m_meanMarker.pose = fb->pose;
		m_markerServer.insert(m_meanMarker);
		m_markerServer.applyChanges();
	}
}

}
