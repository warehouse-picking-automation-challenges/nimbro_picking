// KeyFrame editor for rviz
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "keyframe_editor.h"
#include "limit_delegate.h"

#include <rviz/display_context.h>
#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <rviz/display_factory.h>
#include <rviz/robot/robot_link.h>
#include <rviz/robot/robot_joint.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <nimbro_keyframe_server/GetMotion.h>
#include <nimbro_keyframe_server/SetMotion.h>
#include <tf/transform_listener.h>
#include <moveit/transforms/transforms.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <plot_msgs/Plot.h>

#include <ros/package.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

#include <QThread>
#include <QDebug>
#include <QPushButton>
#include <QMessageBox>
#include <QButtonGroup>
#include <QLabel>
#include <QInputDialog>

// QItemSelection is declared in Qt5
#if QT_VERSION < 0x050000
Q_DECLARE_METATYPE(QItemSelection)
#endif

Q_DECLARE_METATYPE(sensor_msgs::JointStateConstPtr);
Q_DECLARE_METATYPE(nimbro_keyframe_server::PlayMotionFeedbackConstPtr);

#define STREAM_MOTION_TIME_IN_MILLIS 320

namespace nimbro_keyframe_editor
{

static bool hasEnding (std::string const &fullString, std::string const &ending)
{
	if(fullString.length() < ending.length())
		return false;

	return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
}

/**
 * Checks recursively if the given link has an direct or indirect parent link which ends with the parentNameSuffix
 */
static bool isIndirectChildOf(boost::shared_ptr<const urdf::Link> link, std::string parentNameSuffix)
{
		boost::shared_ptr<urdf::Link> parentLink = link->getParent();

		if(parentLink)
		{
			if(hasEnding(parentLink->name, parentNameSuffix))
			{
				return true;
			}
			else
			{
				return isIndirectChildOf(parentLink, parentNameSuffix);
			}
		}
		else
		{
			return false;
		}
}

KeyFrameEditor::KeyFrameEditor()
 : m_group(0)
{
}

KeyFrameEditor::~KeyFrameEditor()
{
	// Need to delete this before the RobotModelLoader is deleted
	delete m_keyFrameView;
	delete m_jointSpaceView;

	delete m_streamMotionTimer;
}

void KeyFrameEditor::onInitialize()
{
	rviz::Display::onInitialize();

	ros::NodeHandle nh("~");

	m_w = new QWidget;
	m_ui.setupUi(m_w);
	setAssociatedWidget(m_w);

	m_ui.motionView->setModel(&m_motionModel);
	m_ui.motionView->setItemDelegate(new LimitDelegate(this));

	connect(m_ui.duplicateButton, SIGNAL(clicked(bool)), SLOT(duplicate()));
	connect(m_ui.deleteButton, SIGNAL(clicked(bool)), SLOT(remove()));
	connect(m_ui.saveButton, SIGNAL(clicked(bool)), SLOT(save()));
	connect(m_ui.playButton, SIGNAL(clicked(bool)), SLOT(play()));
	connect(m_ui.revertButton, SIGNAL(clicked(bool)), SLOT(revert()));
	connect(m_ui.setFromBotButton, SIGNAL(clicked(bool)), SLOT(setFromBot()));
	connect(m_ui.testMotionButton, SIGNAL(clicked(bool)), SLOT(testMotion()));
	connect(m_ui.playFrameButton, SIGNAL(clicked(bool)), SLOT(playFrame()));

	m_loader.reset(new robot_model_loader::RobotModelLoader("robot_description"));
	m_model = m_loader->getModel();

	m_vis.reset(new moveit_rviz_plugin::RobotStateVisualization(getSceneNode(), context_, "Req", NULL));
	m_vis->load(*m_loader->getURDF());
	m_vis->setVisualVisible(true);
	m_vis->setCollisionVisible(false);
	std_msgs::ColorRGBA color;
	color.r = 1.0;
	color.g = 0.0;
	color.b = 0.0;
	color.a = 1.0;
	m_vis->setDefaultAttachedObjectColor(color);

	m_state.reset(new robot_state::RobotState(m_model));
	m_state->setToDefaultValues();

	// Initialize to SRDF init pose
	for(auto group : m_model->getJointModelGroups())
		m_state->setToDefaultValues(group, "init");

	m_state->updateLinkTransforms();

	m_planningScene.reset(new planning_scene::PlanningScene(m_model));

	m_currentRobotState.reset(new robot_state::RobotState(m_model));
	m_currentRobotState->setToDefaultValues();
	m_currentRobotState->updateLinkTransforms();

	m_vis->update(m_state);
	m_vis->setVisible(this->isEnabled());

	rviz::Robot* robot = &m_vis->getRobot();
	const std::map<std::string, rviz::RobotLink*>& links = robot->getLinks();
	for(std::map<std::string, rviz::RobotLink*>::const_iterator it = links.begin(); it != links.end(); ++it)
	{
		rviz::RobotLink* link = it->second;

		boost::shared_ptr<const urdf::Link> urdf_link = m_model->getURDF()->getLink(link->getName());

		if(isIndirectChildOf(urdf_link, "_hand_link"))
		{
			link->setRobotAlpha(0.8);

			if(urdf_link->visual && urdf_link->visual->material)
			{
				urdf::Color color = urdf_link->visual->material->color;

				const float alpha = 0.5;
				link->setColor(alpha * color.r, alpha * color.g, alpha * color.b);
				m_linkColors[link->getName()] = std::make_tuple(alpha * color.r, alpha * color.g, alpha * color.b);
			}
		}
		else
		{
			// If we do not set a color explicitly, the visualization uses the color specified in the URDF
			link->setColor(0.8, 0.8, 0.0);
			m_linkColors[link->getName()] = std::make_tuple(0.8, 0.8, 0.0);
		}
	}

	// Create interactive markers
// 	m_intDisplay = context_->getDisplayFactory()->make("rviz/InteractiveMarkers");
// 	m_intDisplay->initialize(context_);

	m_keyFrameView = new IKView(m_state, m_vis->getRobot().getDisplayContext()->getFrameManager()->getTFClientPtr(), this);
	m_keyFrameView->setModel(&m_motionModel);
	m_keyFrameView->setSelectionModel(m_ui.motionView->selectionModel());

	connect(m_keyFrameView, SIGNAL(changed()), SLOT(updateFromState()));

	m_jointSpaceView = new JointSpaceView(m_state, this);
	m_jointSpaceView->setModel(&m_motionModel);
	m_jointSpaceView->setSelectionModel(m_ui.motionView->selectionModel());

	connect(m_jointSpaceView, SIGNAL(changed()), SLOT(updateFromState()));

	nimbro_keyframe_server::KeyFrame teleFrame;
	teleFrame.setLabel("teleoperation");
	teleFrame.setCartLinearVelocity(0.2);
	teleFrame.setAngularVelocity(0.2);
	teleFrame.setLinearAcceleration(0.2);
	teleFrame.setAngularAcceleration(0.2);
	teleFrame.setJointSpaceVelocity(0.2);
	teleFrame.setJointSpaceAcceleration(0.2);
	m_streamMotion.push_back(teleFrame);

	m_streamMotionTimer = new QTimer(this);
    connect(m_streamMotionTimer, SIGNAL(timeout()), this, SLOT(streamMotionCb()));

	connect(m_ui.motionBox, SIGNAL(currentIndexChanged(QString)), SLOT(loadMotion(QString)));
	connect(m_ui.streamCheckbox, SIGNAL(clicked(bool)), this, SLOT(toggleStream()));
	connect(m_ui.onlineMotion, SIGNAL(clicked(bool)), this, SLOT(streamMotion()));

	//FIXME disable stream motion.
	m_ui.onlineMotion->setEnabled(false);

	m_keyframeServer = nh.resolveName("keyframe_server");

	m_sub_motionList = nh.subscribe(m_keyframeServer + "/motionList", 1, &KeyFrameEditor::handleMotionList, this);

	connect(m_ui.motionView->selectionModel(), SIGNAL(selectionChanged(QItemSelection,QItemSelection)), this, SLOT(handleChangedSelection(QItemSelection)));

	int row = 0;
	m_ui.moveGroupLayout->addWidget(new QLabel("cartesian", m_ui.moveGroupScrollbox), row, 1);
	m_ui.moveGroupLayout->addWidget(new QLabel("joint space", m_ui.moveGroupScrollbox), row, 2);
	m_ui.moveGroupLayout->addWidget(new QLabel("none",m_ui.moveGroupScrollbox), row, 3);

	for(const std::string& group : m_model->getJointModelGroupNames())
	{
		auto jointGroup = m_model->getJointModelGroup(group);

		m_groupNames.push_back(group);
		QLabel* groupLabel = new QLabel(QString::fromStdString(group), m_ui.moveGroupScrollbox);

		QRadioButton* isJointSpaceBtn = new QRadioButton(m_ui.moveGroupScrollbox);
		QRadioButton* isNoneBtn = new QRadioButton(m_ui.moveGroupScrollbox);
		QButtonGroup* buttonGroup = new QButtonGroup(m_ui.moveGroupScrollbox);

		isJointSpaceBtn->setProperty("group",QString::fromStdString(group));
		isNoneBtn->setProperty("group",QString::fromStdString(group));

		isJointSpaceBtn->setProperty("interpolation_space", nimbro_keyframe_server::KeyFrame::IS_JOINT_SPACE);
		isNoneBtn->setProperty("interpolation_space", nimbro_keyframe_server::KeyFrame::IS_NONE);

		buttonGroup->addButton(isJointSpaceBtn);
		buttonGroup->addButton(isNoneBtn);


		m_ui.moveGroupLayout->addWidget(groupLabel, ++row, 0);

		m_ui.moveGroupLayout->addWidget(isJointSpaceBtn, row, 2);
		m_ui.moveGroupLayout->addWidget(isNoneBtn, row, 3);

		connect(isJointSpaceBtn, SIGNAL(clicked(bool)), this, SLOT(handleMoveGroupButtons(bool)));
		connect(isNoneBtn, SIGNAL(clicked(bool)), this, SLOT(handleMoveGroupButtons(bool)));

		m_groupButtons[group][nimbro_keyframe_server::KeyFrame::IS_JOINT_SPACE] = isJointSpaceBtn;
		m_groupButtons[group][nimbro_keyframe_server::KeyFrame::IS_NONE] = isNoneBtn;

		if(jointGroup->getSolverInstance())
		{
			QRadioButton* isCartesianBtn = new QRadioButton(m_ui.moveGroupScrollbox);
			isCartesianBtn->setProperty("group", QString::fromStdString(group));
			isCartesianBtn->setProperty("interpolation_space", nimbro_keyframe_server::KeyFrame::IS_CARTESIAN);

			buttonGroup->addButton(isCartesianBtn);
			m_ui.moveGroupLayout->addWidget(isCartesianBtn, row, 1);
			connect(isCartesianBtn, SIGNAL(clicked(bool)), this, SLOT(handleMoveGroupButtons(bool)));
			m_groupButtons[group][nimbro_keyframe_server::KeyFrame::IS_CARTESIAN] = isCartesianBtn;
		}
		else
			m_groupButtons[group][nimbro_keyframe_server::KeyFrame::IS_CARTESIAN] = 0;
	}

	qRegisterMetaType<QItemSelection>();
	qRegisterMetaType<sensor_msgs::JointStateConstPtr>();
	qRegisterMetaType<nimbro_keyframe_server::PlayMotionFeedbackConstPtr>();

	connect(this,
		SIGNAL(jointStateReceived(sensor_msgs::JointStateConstPtr)),
		SLOT(handleJointState(sensor_msgs::JointStateConstPtr)),
		Qt::QueuedConnection
	);

	m_sub_js = nh.subscribe("/joint_states", 1, &KeyFrameEditor::jointStateReceived, this);

	m_pub_plot = nh.advertise<plot_msgs::Plot>("/plot", 10);
	m_markerPub = nh.advertise<visualization_msgs::Marker>( "/nimbro_keyframe_editor/jointLimitMarker", 0 );

	initMappings();

	m_disableKeyframes = false;

	// we want to set the motion in the motion test display which is embedded into this keyframe editor
	m_setMotionServiceName = nh.resolveName("setMotion");

	m_pub_referencePose = nh.advertise<visualization_msgs::Marker>(
		"/nimbro_keyframe_editor/reference_pose_marker", 0, true
	);

	m_play_motion_action_client.reset(
		new actionlib::SimpleActionClient<nimbro_keyframe_server::PlayMotionAction>(nh, m_keyframeServer + "/play_motion", false)
	);

	QTimer* actTimer = new QTimer(this);
	actTimer->setInterval(100);
	connect(actTimer, SIGNAL(timeout()), SLOT(checkActionClient()));
	actTimer->start();

	connect(this, SIGNAL(actionClientFinished()), SLOT(checkActionClient()), Qt::QueuedConnection);
	connect(this,
		SIGNAL(actionClientFeedback(nimbro_keyframe_server::PlayMotionFeedbackConstPtr, int)),
		SLOT(handleActionFeedback(nimbro_keyframe_server::PlayMotionFeedbackConstPtr, int)),
		Qt::QueuedConnection
	);
}

void KeyFrameEditor::checkActionClient()
{
	if(!m_play_motion_action_client->isServerConnected())
	{
		m_ui.playButton->setText("[not connected]");
		m_ui.playButton->setEnabled(false);

		m_ui.playFrameButton->setText("[not connected]");
		m_ui.playFrameButton->setEnabled(false);

		return;
	}

	m_ui.playButton->setEnabled(true);
	m_ui.playFrameButton->setEnabled(true);

	if(m_actionActive)
	{
		auto state = m_play_motion_action_client->getState();

		if(state.isDone())
		{
			ROS_INFO("Action finished. State: %s", state.toString().c_str());
			m_actionActive = false;
		}
	}

	if(m_actionActive)
	{
		m_ui.playButton->setText("STOP");
		m_ui.playFrameButton->setText("STOP");
	}
	else
	{
		m_ui.playButton->setText("Play");
		m_ui.playFrameButton->setText("Play Frame");
		m_motionModel.resetFeedback();
	}
}

void KeyFrameEditor::handleActionFeedback(const nimbro_keyframe_server::PlayMotionFeedbackConstPtr& feedback, int kf_offset)
{
	nimbro_keyframe_server::ExecutionFeedback fb = feedback->player_feedback;
	fb.keyframe_index += kf_offset;
	m_motionModel.setFeedback(fb);
	m_ui.motionView->selectRow(fb.keyframe_index);
}

void KeyFrameEditor::onEnable()
{
	m_vis->setVisualVisible(true);
	m_vis->setCollisionVisible(false);
	m_vis->setVisible(true);
}

int KeyFrameEditor::selectedKeyFrameIndex()
{
	if(m_disableKeyframes)
		return 0;
	QModelIndexList list = m_ui.motionView->selectionModel()->selectedRows();
	if (list.size() != 0)
		return list[0].row();
	else
		return -1;
}

void KeyFrameEditor::handleMoveGroupButtons(bool checked)
{
	if (!checked || !sender())
		return;
	QRadioButton* btn = qobject_cast< QRadioButton* >(sender());
	nimbro_keyframe_server::KeyFrame::InterpolationSpace space =
	(nimbro_keyframe_server::KeyFrame::InterpolationSpace)btn->property("interpolation_space").toInt();

	std::string groupName = btn->property("group").toString().toStdString();

// 	ROS_INFO_STREAM("KeyFrameEditor::handleMoveGroupButtons: "<<groupName<<" "<<space);

	int idx = selectedKeyFrameIndex();
	if(idx != -1)
	{
		m_motionModel.motion()[idx].setInterpolationSpace(
			groupName,
			space
		);
	}

	m_keyFrameView->updateActiveGroups();
	m_jointSpaceView->updateActiveGroups();

	// We need to update once at this point, to transfer the current state into the keyframe
	updateFromState();
}

void KeyFrameEditor::streamMotionCb()
{
	nimbro_keyframe_server::SetMotion setMotion;
	setMotion.request.play = true;
	setMotion.request.save = false;

	const nimbro_keyframe_server::Motion& motion = m_motionModel.motion();

	setMotion.request.motion = motion.toMsg();
	if(!ros::service::call(m_keyframeServer + "/setMotion", setMotion))
	{
		ROS_ERROR("Service set_motion NOT FOUND");
		return;
	}
}

void KeyFrameEditor::handleChangedSelection(const QItemSelection& selection)
{
	if(selection.count() == 0)
		return;

	int idx = selection[0].topLeft().row();

	for(const std::string& group : m_groupNames)
	{
		nimbro_keyframe_server::KeyFrame* frame = &m_motionModel.motion()[idx];
		if (!frame->hasJointGroup(group))
		{
			frame->setInterpolationSpace(
				group,
				nimbro_keyframe_server::KeyFrame::IS_NONE
			);
		}
		nimbro_keyframe_server::KeyFrame::InterpolationSpace space;
		space = frame->interpolationSpace(group);

		auto btn = m_groupButtons[group][space];
		if(btn)
			btn->setChecked(true);
	}

	updateFromModel();
}

void KeyFrameEditor::handleMotionList(const nimbro_keyframe_server::MotionList& msg)
{
	if (m_ui.streamCheckbox->isChecked())
		return;

	m_ui.motionBox->clear();
	for(size_t i = 0; i < msg.motions.size(); ++i)
		m_ui.motionBox->addItem(QString::fromStdString(msg.motions[i]));

	m_ui.motionBox->addItem("[new]");

	m_ui.motionBox->setCurrentIndex(0);
	ROS_INFO("Motion list processed");
}

void KeyFrameEditor::toggleStream()
{
	if(m_ui.streamCheckbox->isChecked())
	{
		m_ui.motionBox->setEnabled(false);
		m_motionModel.load(m_streamMotion.toMsg());
		m_ui.motionView->selectRow(0);
	}
	else
	{
		m_ui.motionBox->setEnabled(true);
		loadMotion(m_ui.motionBox->currentText());
	}
}

void KeyFrameEditor::streamMotion()
{
	if(m_ui.onlineMotion->isChecked())
	{
		m_disableKeyframes = true;
		loadMotion("");
		setCurrentStateAsMotion();
		m_keyFrameView->disableKeyframes(true);
		m_jointSpaceView->disableKeyframes(true);
		m_ui.motionView->selectRow(0);

		m_streamMotionTimer->start(STREAM_MOTION_TIME_IN_MILLIS);
	}
	else
	{
		m_jointSpaceView->disableKeyframes(false);
		m_keyFrameView->disableKeyframes(false);
		m_ui.motionBox->setEnabled(true);
		loadMotion(m_ui.motionBox->currentText());
		m_disableKeyframes = false;

		m_streamMotionTimer->stop();
	}
}

void KeyFrameEditor::setCurrentStateAsMotion()
{
	nimbro_keyframe_server::Motion motion("StreamMotion");

	nimbro_keyframe_server::KeyFrame frame;
	frame.setLabel("frame0");
	frame.setCartLinearVelocity(0.3);
	frame.setAngularVelocity(0.4);
	motion.push_back(frame);

	// And load our new motion
	m_motionModel.load(motion);
	*m_state = *m_currentRobotState;
	updateFromState();

}

void KeyFrameEditor::loadMotion(const QString& name)
{
	if (m_ui.streamCheckbox->isChecked())
		return;

	nimbro_keyframe_server::Motion motion;

	if(name.isEmpty())
		m_motionModel.clear();
	else if(name == "[new]")
	{
		QString name = QInputDialog::getText(m_w, "Motion name",
			"Please enter a name (valid filename) for the motion"
		);

		if(name.isNull())
			return;

		nimbro_keyframe_server::Motion new_motion(name.toStdString());

		nimbro_keyframe_server::KeyFrame frame;
		frame.setLabel("frame0");
		frame.setCartLinearVelocity(0.1);
		frame.setAngularVelocity(0.1);
		frame.setLinearAcceleration(0.2);
		frame.setAngularAcceleration(0.2);
		frame.setJointSpaceVelocity(0.1);
		frame.setJointSpaceAcceleration(0.2);
		new_motion.push_back(frame);

		// Uncheck "teleoperation" without triggering the slot
		m_ui.streamCheckbox->blockSignals(true);
		m_ui.streamCheckbox->setChecked(false);
		m_ui.streamCheckbox->blockSignals(false);

		// And load our new motion
		motion = new_motion;
	}
	else
	{
		// Fetch the selected motion from the server
		nimbro_keyframe_server::GetMotion srv;
		srv.request.name = name.toStdString();

		if(!ros::service::call(m_keyframeServer + "/getMotion", srv))
		{
			ROS_ERROR("Could not get motion from keyframe server");
			return;
		}

		motion = nimbro_keyframe_server::Motion::loadFromMsg(srv.response.motion);
	}

	m_motionModel.load(motion);
	m_ui.motionView->selectRow(0);
	publishReferenceObjectPose(
		motion.referencePose,
		motion.referenceObjectMesh
	);
}

void KeyFrameEditor::update(float wall_dt, float ros_dt)
{
	// move the robot visualization into the fixed frame
	rviz::FrameManager* frameManager = context_->getFrameManager();
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	frameManager->getTransform(m_model->getModelFrame(), ros::Time(0), position, orientation);
	m_vis->getRobot().setPosition(position);
	m_vis->getRobot().setOrientation(orientation);

	rviz::Display::update(wall_dt, ros_dt);
}

void KeyFrameEditor::duplicate()
{
	int idx = m_ui.motionView->selectionModel()->currentIndex().row();
	if(idx < 0)
		return;

	m_motionModel.duplicateKeyFrame(idx);
}

void KeyFrameEditor::remove()
{
	int idx = m_ui.motionView->selectionModel()->currentIndex().row();
	if(idx < 0)
		return;

	m_motionModel.removeKeyFrame(idx);
}

void KeyFrameEditor::save()
{
	const nimbro_keyframe_server::Motion& motion = m_motionModel.motion();

	nimbro_keyframe_server::SetMotion srv;
	srv.request.motion = motion.toMsg();
	srv.request.play = false;
	srv.request.save = true;

	if(!ros::service::call(m_keyframeServer + "/setMotion", srv))
	{
		QMessageBox::critical(0, "Warning", "Could not save motion");
		return;
	}

	QMessageBox::information(0, "Information", "Motion saved.");
}

void KeyFrameEditor::play()
{
	if(!m_actionActive)
	{
		nimbro_keyframe_server::PlayMotionGoal goal;
		goal.motion_msg = m_motionModel.motion().toMsg();
		goal.use_existing_motion = false;

		m_play_motion_action_client->sendGoal(goal,
			boost::bind(&KeyFrameEditor::actionClientFinished, this),
			ActionClient::SimpleActiveCallback(),
			boost::bind(&KeyFrameEditor::actionClientFeedback, this, _1, 0)
		);
		m_actionActive = true;
		checkActionClient();

		ROS_INFO("Motion requested");
	}
	else
		m_play_motion_action_client->cancelAllGoals();
}

void KeyFrameEditor::playFrame()
{
	if(!m_actionActive)
	{
		if(selectedKeyFrameIndex() < 0)
			return;

		nimbro_keyframe_server::PlayMotionGoal goal;
		goal.motion_msg = m_motionModel.motion().toMsg();
		goal.use_existing_motion = false;

		nimbro_keyframe_server::KeyFrameMsg key = goal.motion_msg.keyframes[selectedKeyFrameIndex()];
		ROS_INFO("Selected Keyframe has number: %d", selectedKeyFrameIndex());
		goal.motion_msg.keyframes.clear();
		goal.motion_msg.keyframes.push_back(key);

		m_play_motion_action_client->sendGoal(goal,
			boost::bind(&KeyFrameEditor::actionClientFinished, this),
			ActionClient::SimpleActiveCallback(),
			boost::bind(&KeyFrameEditor::actionClientFeedback, this, _1, (int)selectedKeyFrameIndex())
		);
		m_actionActive = true;
		checkActionClient();

		ROS_INFO("playFrame requested");
	}
	else
		m_play_motion_action_client->cancelAllGoals();
}


void KeyFrameEditor::revert()
{
	loadMotion(m_ui.motionBox->currentText());
}

void KeyFrameEditor::setFromBot()
{
	*m_state = *m_currentRobotState;
	updateFromState();
}

void KeyFrameEditor::testSelfCollision()
{
// 	ROS_INFO_STREAM("KeyFrameEditor::testSelfCollision() ------ ");

// 	ros::Time begin = ros::Time::now();
	m_planningScene->setCurrentState(*m_state);

	collision_detection::CollisionRequest collision_request;
	collision_request.contacts = true;
	collision_request.max_contacts = 1000;
	collision_detection::CollisionResult collision_result;

	collision_detection::CollisionRobotPtr collisionRobot  = m_planningScene->getCollisionRobotNonConst();
	collisionRobot->setPadding(0.01);

 	collisionRobot->checkSelfCollision(collision_request, collision_result, m_planningScene->getCurrentStateNonConst(), m_planningScene->getAllowedCollisionMatrix());
// 	ros::Duration elapsedTime = ros::Time::now() - begin;
// 
// 	ROS_INFO_STREAM("Current state is "
//                 << (collision_result.collision ? "in" : "not in")
//                 << " self collision");
// 	ROS_INFO_STREAM("elapsedTime: "
//                 << (elapsedTime.toSec()*1000)
//                 << " ms");

	collision_detection::CollisionResult::ContactMap contactMap = collision_result.contacts;
// 	ROS_INFO_STREAM("contactMap "<<(contactMap.empty() ? "EMPTY" : "not EMPTY"));

	// reset the color of all links which previously have been in a collision
	if(!m_linkInCollision.empty())
	{
			for(auto& kv : m_linkInCollision)
			{
				const std::string linkName = kv.first;
				m_vis->getRobot().getLink(linkName)->setColor(std::get<0>(m_linkColors[linkName]), std::get<1>(m_linkColors[linkName]), std::get<2>(m_linkColors[linkName]));
			}
	}
	m_linkInCollision.clear();

	collision_detection::CollisionResult::ContactMap::const_iterator it;
	for(it = collision_result.contacts.begin();
		it != collision_result.contacts.end();
		++it)
	{
// 	ROS_INFO("Contact between: %s and %s",
// 			it->first.first.c_str(),
// 			it->first.second.c_str());

		std::string linkName = it->first.first.c_str();
		m_vis->getRobot().getLink(linkName)->setColor(1.0, 0.0, 0.0);
		m_linkInCollision[linkName] = true;

		linkName = it->first.second.c_str();
		m_vis->getRobot().getLink(linkName)->setColor(1.0, 0.0, 0.0);
		m_linkInCollision[linkName] = true;
	}
}


void KeyFrameEditor::testMotion()
{
	if(!ros::service::exists(m_setMotionServiceName, true))
	{
		QMessageBox::critical(0, "Warning", "The test motion display is not loaded");
		return;
	}

	const nimbro_keyframe_server::Motion& motion = m_motionModel.motion();

	nimbro_keyframe_server::SetMotion srv;
	srv.request.motion = motion.toMsg();
	srv.request.play = true;
	srv.request.save = false;

	/**
	 * HACK: The service call can only be executed if ros::spin was called. Since
	 *       the service is advertised within the same process, we need to call the
	 *       service in its own thread and call ros::spin manually while the call
	 *       blocks.
	 */
	volatile bool finished = false;
	volatile bool success = false;
	boost::thread thread([&]() {
		success = ros::service::call(m_setMotionServiceName, srv);
		finished = true;
	});

	while(!finished)
	{
		ros::spinOnce();
		usleep(1000);
	}
	thread.join();

	if(!success)
	{
		QMessageBox::critical(0, "Warning", "Could not test motion");
		return;
	}
}

void KeyFrameEditor::updateFromState()
{
	m_state->updateLinkTransforms();
	m_vis->update(m_state);
	m_keyFrameView->updateFromState();
	m_jointSpaceView->updateFromState();

	// We need to transfer the state values to the keyframe...
	int idx = selectedKeyFrameIndex();
	if(idx != -1)
	{
		auto& kf = m_motionModel.motion()[idx];
		auto& groups = kf.jointGroups();

		for(auto& pair : groups)
		{
			auto jointModelGroup = m_model->getJointModelGroup(pair.first);
			if(!jointModelGroup)
				continue;

			switch(pair.second.interpolationSpace())
			{
				case nimbro_keyframe_server::KeyFrame::IS_JOINT_SPACE:
				{
					std::vector<double> positions(jointModelGroup->getVariableCount());
					auto names = jointModelGroup->getVariableNames();

					m_state->copyJointGroupPositions(jointModelGroup, positions);

					for(unsigned int i = 0; i < positions.size(); ++i)
						kf.setJointPosition(names[i], positions[i]);

					break;
				}
				case nimbro_keyframe_server::KeyFrame::IS_CARTESIAN:
				{
					std::string tip = jointModelGroup->getSolverInstance()->getTipFrame();
					kf.setState(pair.first, m_state->getGlobalLinkTransform(tip));
					break;
				}
				case nimbro_keyframe_server::KeyFrame::IS_NONE:
					break;	
			}
		}
	}

	// Publish points for the plotter
	plot_msgs::Plot plot;

	for(auto joint : m_model->getJointModels())
	{
		plot_msgs::PlotPoint point;
		point.name = "/nimbro_keyframe_editor/" + joint->getName();
		point.value = m_state->getJointPositions(joint)[0];

		plot.points.push_back(point);
	}

	plot.header.stamp = ros::Time::now();

	m_pub_plot.publish(plot);


	// Publish joint limit markers
	for(unsigned int i=0; i<m_model->getJointModels().size(); ++i)
	{
		if(m_ignore[i])
			continue;

		moveit::core::JointModel* joint = m_model->getJointModels()[i];
		std::string jointName = joint->getName();

		double q = m_state->getJointPositions(joint)[0];

		// calculate the distance from the joint limit and compute the intensity of the red color channnel for the marker
		const double limitStartDiff = 0.2 * M_PI;
		double r = 0.0;
		double maxJointAngleDiff = q - (m_jointNameToUpperLimit[jointName] - limitStartDiff);
		if(maxJointAngleDiff > 0)
		{
			r = maxJointAngleDiff/limitStartDiff;
		}

		double minJointAngleDiff = q - (m_jointNameToLowerLimit[jointName] + limitStartDiff);
		if(minJointAngleDiff < 0)
		{
			r = minJointAngleDiff/limitStartDiff;
		}
		r = std::abs(r);

		// Get pose of Joint for marker
		const Eigen::Vector3d& joint_axis = ((moveit::core::RevoluteJointModel*)joint)->getAxis();
		Eigen::Affine3d transform = m_state->getFrameTransform(joint->getChildLinkModel()->getName());
		Eigen::Quaterniond orientation;
		orientation.setFromTwoVectors(Eigen::Vector3d::UnitZ(), joint_axis);
		transform = transform.rotate( orientation );

		visualization_msgs::Marker marker;
		tf::poseEigenToMsg(transform, marker.pose);

		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = m_jointNameToJointGroupName[jointName];
		marker.id = i;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = 0.07;
		marker.scale.y = 0.07;
		marker.scale.z = 0.07;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = r;
		marker.color.g = 1.0-r;
		marker.color.b = 0.0;
		m_markerPub.publish( marker );
	}

	testSelfCollision();
}

void KeyFrameEditor::handleJointState(const sensor_msgs::JointStateConstPtr& msg)
{
	if(msg->position.size() != msg->name.size())
		return;

	for(unsigned int i = 0; i < msg->name.size(); ++i)
	{
		if(!m_model->hasJointModel(msg->name[i]))
			continue;

		double pos = msg->position[i];
		m_currentRobotState->setJointPositions(msg->name[i], &pos);
	}

}

void KeyFrameEditor::updateFromModel()
{
	const auto kf = m_motionModel.motion()[selectedKeyFrameIndex()];

	for(auto pair : kf.jointGroups())
	{
		auto kfGroup = pair.second;

		auto group = m_model->getJointModelGroup(pair.first);
		if(!group)
			continue;

		switch(kfGroup.interpolationSpace())
		{
			case nimbro_keyframe_server::KeyFrame::IS_CARTESIAN:
			{
				if(!m_state->setFromIK(group, kfGroup.state()))
				{
					ROS_INFO_STREAM("idx = " << selectedKeyFrameIndex() << ", state:\n" << kfGroup.state().matrix());
					ROS_WARN("Editor: IK failed while loading frame '%s', group %s", kf.label().c_str(), pair.first.c_str());
					m_state->setToDefaultValues(group, "");
				}

				break;
			}
			case nimbro_keyframe_server::KeyFrame::IS_JOINT_SPACE:
			{
				const auto& names = group->getVariableNames();
				const auto& positions = kf.jointPositions();

				for(auto name : names)
				{
					auto it = positions.find(name);
					if(it != positions.end())
					{
						double angle = it->second;
						m_state->setVariablePosition(name, angle);
					}
				}
				break;
			}
			case nimbro_keyframe_server::KeyFrame::IS_NONE:
				break;
		}
	}

	m_state->updateLinkTransforms();
	m_vis->update(m_state);
	m_keyFrameView->updateActiveGroups();
	m_jointSpaceView->updateActiveGroups();
}

void KeyFrameEditor::initMappings()
{
	m_ignore.clear();
	m_jointNameToJointGroupName.clear();
	m_jointNameToLowerLimit.clear();
	m_jointNameToUpperLimit.clear();

	for(auto joint : m_model->getJointModels())
	{
		std::string jointName = joint->getName();

		boost::shared_ptr<const urdf::Joint> urdfJoint = m_model->getURDF()->getJoint(jointName);

		m_ignore.push_back(!urdfJoint || !urdfJoint->limits);
		if(!urdfJoint || !urdfJoint->limits)
		{
			continue;
		}

		// store the lower limit of the joint
		m_jointNameToLowerLimit[jointName] = urdfJoint->limits->lower;

		// store the upper limit of the joint
		m_jointNameToUpperLimit[jointName] = urdfJoint->limits->upper;

		// find the name of the joint group to which this joint belongs
		bool groupFound = false;
		for(moveit::core::JointModelGroup* group : m_model->getJointModelGroups())
		{
			std::vector<std::string> jointModelNames = group->getJointModelNames();
			if(std::find(jointModelNames.begin(), jointModelNames.end(), jointName) != jointModelNames.end())
			{
				m_jointNameToJointGroupName[jointName] = group->getName();
				groupFound = true;
				break;
			}
		}

		if(!groupFound)
			m_ignore.back() = true;
	}


}

void KeyFrameEditor::publishReferenceObjectPose(
	const Eigen::Affine3d& pose,
	const std::string& mesh
)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();

	if(!std::isfinite(pose.translation().x()) ||
	   mesh.empty())
	{
		marker.action = visualization_msgs::Marker::DELETE;
		m_pub_referencePose.publish(marker);
		return;
	}

	tf::poseEigenToMsg(pose, marker.pose);

	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = mesh;
	marker.color.a = 1.0;
	marker.color.b = 1;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	m_pub_referencePose.publish(marker);
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_keyframe_editor::KeyFrameEditor, rviz::Display)
