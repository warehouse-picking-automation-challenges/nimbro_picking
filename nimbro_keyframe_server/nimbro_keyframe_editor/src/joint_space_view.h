// Joint space view
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef JOINT_SPACE_VIEW_H
#define JOINT_SPACE_VIEW_H

#include <QObject>
#include <QItemSelectionModel>

#include "motion_model.h"

#ifndef Q_MOC_RUN
#include <interactive_markers/interactive_marker_server.h>

#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#endif

namespace nimbro_keyframe_editor
{

class JointSpaceView : public QObject
{
Q_OBJECT
public:
	explicit JointSpaceView(const robot_state::RobotStatePtr& state, QObject* parent = 0);
	virtual ~JointSpaceView();

	void setModel(MotionModel* model);
	void setSelectionModel(QItemSelectionModel* model);
	void disableKeyframes(bool disable);
	void updateActiveGroups();
public Q_SLOTS:
	void updateFromState();
private Q_SLOTS:
	void handleSelection(const QItemSelection& selection);
Q_SIGNALS:
	void changed();
private:
	robot_model::RobotModelConstPtr m_robotModel;
	robot_state::RobotStatePtr m_robotState;

	MotionModel* m_model;
	QItemSelectionModel* m_selectionModel;

	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> m_markerServer;
	std::map<std::string, Eigen::Matrix3d> m_jointAxisTransforms;
	std::map<std::string, Eigen::Affine3d> m_markerPose;

	std::vector<std::string> m_groups;

	int m_modelRow;

	void processMarker(const moveit::core::JointModel* joint, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);
	void processPrismaticMarker(const moveit::core::JointModel* joint, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);
	bool m_disabledKeyframes;
};


}


#endif
