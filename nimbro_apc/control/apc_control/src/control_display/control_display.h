// rqt plugin displaying the items inside the shelf and tote
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef CONTROL_DISPLAY_H
#define CONTROL_DISPLAY_H

#include <rqt_gui_cpp/plugin.h>

#include <ros/subscriber.h>
#include <QTimer>

#include "ui_control_display.h"

#include <apc_control/ShelfState.h>

namespace apc_control
{

class ControlDisplay : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	ControlDisplay();
	virtual ~ControlDisplay();

	virtual void initPlugin(qt_gui_cpp::PluginContext& ctx) override;
	virtual void shutdownPlugin() override;

Q_SIGNALS:
	void shelfStateReceived(const apc_control::ShelfStateConstPtr& msg);
private Q_SLOTS:
	void handleShelfState(const apc_control::ShelfStateConstPtr& msg);
	void timerTriggered();
	void handleStart();
private:
	QWidget* m_w;
	Ui_ControlDisplay m_ui;

	QTimer* m_timer;

	ros::Subscriber m_sub_state;

	int m_currentWorkItemIdx;
	std::vector<ros::Duration> m_item_times;

	bool m_attempt_running;
	ros::Time m_start_attempt;
	ros::Time m_start_item;

	QString durationToString(ros::Duration d);

};

}

#endif
