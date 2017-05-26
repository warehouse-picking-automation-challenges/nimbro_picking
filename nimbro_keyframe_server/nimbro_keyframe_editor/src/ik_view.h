// Provides markers to edit a RobotState in IK space
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef IK_VIEW_H
#define IK_VIEW_H

#include <QtCore/QObject>

#ifndef Q_MOC_RUN
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#endif


class QItemSelectionModel;
class QItemSelection;

namespace nimbro_keyframe_editor
{

class MotionModel;

class IKView : public QObject
{
Q_OBJECT
public:
	explicit IKView(const robot_state::RobotStatePtr& state, const boost::shared_ptr<tf::Transformer>& transformer, QObject* parent = 0);
	virtual ~IKView();

	void setModel(MotionModel* model);
	void setSelectionModel(QItemSelectionModel* selectionModel);
	void disableKeyframes(bool disable);
Q_SIGNALS:
	void changed();
	void triggerInteractiveUpdate(int index);
public Q_SLOTS:
	void updateActiveGroups();
	void updateFromState();
private Q_SLOTS:
	void modelReset();
	void handleSelection(const QItemSelection& selection);

	void handleInteractiveUpdate(int index);

	void updateMeanMarker();
private:
	struct GroupHandler
	{
		const robot_model::JointModelGroup* group;
		robot_interaction::RobotInteractionPtr interaction;
		robot_interaction::InteractionHandlerPtr handler;
		Eigen::Affine3d meanStartPose;
	};

	void updateFromModel();
	void handleMeanUpdate(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& fb);

	boost::shared_ptr<tf::Transformer> m_transformer;

	MotionModel* m_model;
	QItemSelectionModel* m_selectionModel;

	robot_state::RobotStatePtr m_kfState;
	robot_model::RobotModelConstPtr m_robotModel;

	int m_modelRow;

	std::vector<GroupHandler> m_handlers;
	bool m_disabledKeyframes;

	interactive_markers::InteractiveMarkerServer m_markerServer;
	visualization_msgs::InteractiveMarker m_meanMarker;
	Eigen::Affine3d m_meanMarkerStart;
};

}

#endif
