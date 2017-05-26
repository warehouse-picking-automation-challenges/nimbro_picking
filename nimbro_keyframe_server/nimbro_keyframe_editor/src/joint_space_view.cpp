
#include "joint_space_view.h"
#include <nimbro_keyframe_server/SetMotion.h>
#include <eigen_conversions/eigen_msg.h>
#include <interactive_markers/tools.h>

namespace nimbro_keyframe_editor
{

const float MARKER_SCALE = 0.15;
const float MARKER_SCALE_HAND = 0.1;
const float EPSILON = 0.1;

JointSpaceView::JointSpaceView(const robot_state::RobotStatePtr& state, QObject* parent)
 : QObject(parent)
 , m_robotModel(state->getRobotModel())
 , m_robotState(state)
 , m_modelRow(-1)
{
	m_markerServer.reset(new interactive_markers::InteractiveMarkerServer(
		"momaro/joint_markers"
	));
	m_disabledKeyframes = false;
}

JointSpaceView::~JointSpaceView()
{
}

void JointSpaceView::setModel(MotionModel* model)
{
	m_model = model;
}

void JointSpaceView::setSelectionModel(QItemSelectionModel* model)
{
	m_selectionModel = model;
	connect(model, SIGNAL(selectionChanged(QItemSelection,QItemSelection)), SLOT(handleSelection(QItemSelection)));
}

void JointSpaceView::handleSelection(const QItemSelection& selection)
{
	if(selection.count() != 0)
		m_modelRow = selection[0].topLeft().row();
	else
		m_modelRow = -1;

	updateActiveGroups();
}


void JointSpaceView::updateActiveGroups()
{
	m_markerServer->clear();

	if(m_modelRow != -1)
	{
		updateFromState();
	}

	m_markerServer->applyChanges();
}

static bool hasEnding (std::string const &fullString, std::string const &ending)
{
	if(fullString.length() < ending.length())
		return false;

	return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
}

void JointSpaceView::updateFromState()
{
	if(m_modelRow == -1)
		return;
	
	const auto& kf = m_model->motion()[m_modelRow];

	for(auto jointGroup : kf.jointGroups())
	{
		if(jointGroup.second.interpolationSpace() != nimbro_keyframe_server::KeyFrame::IS_JOINT_SPACE)
			continue;

		const auto group = m_robotModel->getJointModelGroup(jointGroup.first);
		if(!group)
			continue;

		for(const auto joint : group->getJointModels())
		{
			// Ignore mimic joints, they are controlled by other joints.
			if(joint->getMimic())
				continue;

			if(joint->getType() == moveit::core::JointModel::JointType::REVOLUTE)
			{
				visualization_msgs::InteractiveMarker jointMarker;
				jointMarker.header.frame_id = m_robotModel->getModelFrame();
				jointMarker.name = joint->getName();

				// The markers on the hands of the robot should be displayed with a smaller scale
				if(hasEnding(jointGroup.first, "_hand"))
				{
					jointMarker.scale = MARKER_SCALE_HAND;
				}
				else
				{
					jointMarker.scale = MARKER_SCALE;
				}

				// Get pose of Joint for marker
				const auto& joint_axis = ((moveit::core::RevoluteJointModel*)joint)->getAxis();
				Eigen::Affine3d transform = m_robotState->getFrameTransform(joint->getChildLinkModel()->getName());
				Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitX().cross(joint_axis);
				double rot_angle = acos(Eigen::Vector3d::UnitX().dot(joint_axis));

				// Only change marker rotation axis if the marker axis and the rotation axis are not too similar or opposing
				if (m_jointAxisTransforms.find(joint->getName()) == m_jointAxisTransforms.end())
				{
					Eigen::Matrix3d axisTransform = Eigen::Matrix3d::Identity();
					if(fabs((-joint_axis - Eigen::Vector3d::UnitX()).norm()) <= EPSILON)
					{
						axisTransform = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
						transform = transform.rotate( axisTransform );
					}
					else if (fabs((joint_axis - Eigen::Vector3d::UnitX()).norm()) >= EPSILON)
					{
						axisTransform = Eigen::AngleAxisd(rot_angle, rot_axis);
						transform = transform.rotate( axisTransform );
					}
					m_jointAxisTransforms[jointMarker.name] = axisTransform;
				}
				else
					transform = transform.rotate( m_jointAxisTransforms[jointMarker.name] );

				tf::poseEigenToMsg(transform, jointMarker.pose);

				m_markerPose[joint->getName()] = transform;

				visualization_msgs::InteractiveMarkerControl ctrl;
				ctrl.always_visible = true;
				ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

				// If the marker manipulates a joint of the hand, it should be displayed according to the color of the joint in the urdf
				if(hasEnding(jointGroup.first, "_hand"))
				{
					ctrl.orientation.w = 1;
					interactive_markers::makeDisc(jointMarker, ctrl);

					boost::shared_ptr<const urdf::Link> urdf_link = m_robotModel->getURDF()->getLink(joint->getName()+"_link");
					if(urdf_link && urdf_link->visual && urdf_link->visual->material)
					{
						urdf::Color color = urdf_link->visual->material->color;
						for(std::size_t i = 0; i < ctrl.markers[0].colors.size(); ++i)
						{
							ctrl.markers[0].colors[i].r = color.r;
							ctrl.markers[0].colors[i].g = color.g;
							ctrl.markers[0].colors[i].b = color.b;
						}
					}
				}

				jointMarker.controls.push_back(ctrl);

				m_markerServer->insert(jointMarker, boost::bind(&JointSpaceView::processMarker, this, joint, _1));
			}
			else if(joint->getType() == moveit::core::JointModel::PRISMATIC)
			{
				visualization_msgs::InteractiveMarker jointMarker;
				jointMarker.header.frame_id = m_robotModel->getModelFrame();
				jointMarker.name = joint->getName();

				jointMarker.scale = MARKER_SCALE;

				// Get pose of Joint for marker
				const auto& joint_axis = ((moveit::core::PrismaticJointModel*)joint)->getAxis();
				Eigen::Affine3d transform = m_robotState->getFrameTransform(joint->getChildLinkModel()->getName());

				// Create rotation that rotates the X axis onto the joint axis
				Eigen::Matrix3d rot;
				rot.col(0) = joint_axis;
				if(std::abs(rot.col(0).x()) < 0.7)
					rot.col(1) = Eigen::Vector3d::UnitX().cross(rot.col(0));
				else
					rot.col(1) = Eigen::Vector3d::UnitY().cross(rot.col(0));
				rot.col(2) = rot.col(0).cross(rot.col(1));

				m_markerPose[joint->getName()] = transform * Eigen::Affine3d(rot);

				tf::poseEigenToMsg(m_markerPose[joint->getName()], jointMarker.pose);

				visualization_msgs::InteractiveMarkerControl ctrl;
				ctrl.always_visible = true;
				ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

				visualization_msgs::Marker cylinder;
				cylinder.action = visualization_msgs::Marker::ADD;
				cylinder.type = visualization_msgs::Marker::CYLINDER;
				cylinder.scale.x = 0.1;
				cylinder.scale.y = 0.1;
				cylinder.scale.z = 0.1;
				cylinder.color.a = 0.8;
				cylinder.color.r = 1.0;

				cylinder.pose.orientation.w = cos(M_PI/4.0);
				cylinder.pose.orientation.x = 0.0;
				cylinder.pose.orientation.y = sin(M_PI/4.0);
				cylinder.pose.orientation.z = 0.0;

				ctrl.markers.push_back(cylinder);

				jointMarker.controls.push_back(ctrl);

				m_markerServer->insert(jointMarker, boost::bind(&JointSpaceView::processPrismaticMarker, this, joint, _1));
			}
		}
	}

	m_markerServer->applyChanges();
}

void JointSpaceView::processMarker(const moveit::core::JointModel* joint, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb)
{
	Eigen::Affine3d pose;
	tf::poseMsgToEigen(fb->pose, pose);
	Eigen::Matrix3d rot0 = pose.rotation();
	Eigen::Matrix3d rot1 = m_markerPose[fb->marker_name].rotation();
	auto w = rot0.transpose() * rot1;

	double phi = acos((w.trace() - 1)/2);
	if (!std::isfinite(phi))
		return;
	if (w(2,1) - w(1,2) >= 0 )
		phi = -phi;

	double oldPosition = *m_robotState->getJointPositions(joint);
	oldPosition = oldPosition + phi;

	if (joint->satisfiesPositionBounds(&oldPosition))
	{
		m_markerPose[fb->marker_name] = pose;
		ROS_INFO_STREAM("[processMarker] Setting joint :" << joint->getName() << " to: " << oldPosition );
		m_robotState->setJointPositions(joint, &oldPosition);
		m_robotState->update();

		updateFromState();
		changed();
	}
}

void JointSpaceView::processPrismaticMarker(const moveit::core::JointModel* joint, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb)
{
	Eigen::Affine3d pose;
	tf::poseMsgToEigen(fb->pose, pose);

	auto w = m_markerPose[fb->marker_name].inverse() * pose;

	auto axis = ((moveit::core::PrismaticJointModel*)joint)->getAxis();

	int idx;
	w.translation().array().abs().maxCoeff(&idx);
	double amount = w.translation()[idx] / axis[idx];

	double oldPosition = *m_robotState->getJointPositions(joint);
	oldPosition = oldPosition + amount;

	if (joint->satisfiesPositionBounds(&oldPosition))
	{
		m_markerPose[fb->marker_name] = pose;
		ROS_INFO_STREAM("[processMarker] Setting joint :" << joint->getName() << " to: " << oldPosition );
		m_robotState->setJointPositions(joint, &oldPosition);
		m_robotState->update();
	}

	updateFromState();
	changed();
}

void JointSpaceView::disableKeyframes(bool disable)
{
	m_disabledKeyframes = disable;
}
}
