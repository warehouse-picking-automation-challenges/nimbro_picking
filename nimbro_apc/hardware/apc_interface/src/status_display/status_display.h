// rqt plugin displaying the controller status
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef STATUS_DISPLAY_H
#define STATUS_DISPLAY_H

#include <rqt_gui_cpp/plugin.h>

#include <apc_interface/ControllerState.h>

#include <ros/subscriber.h>

#include "ui_status_display.h"

namespace apc_interface
{

class StatusDisplay : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	StatusDisplay();
	virtual ~StatusDisplay();

	virtual void initPlugin(qt_gui_cpp::PluginContext& ctx) override;
	virtual void shutdownPlugin() override;

Q_SIGNALS:
	void controllerStateReceived(const apc_interface::ControllerStateConstPtr& msg);
private Q_SLOTS:
	void handleControllerState(const apc_interface::ControllerStateConstPtr& msg);
	void toggleVacuum();
	void toggleLight();
	void toggleVacuumPower();
	void updateSliderLabel(int ticks);
private:
	QWidget* m_w;
	Ui_StatusDisplay m_ui;

	ros::Subscriber m_sub_state;
	apc_interface::ControllerStateConstPtr m_lastState;
};

}

#endif
