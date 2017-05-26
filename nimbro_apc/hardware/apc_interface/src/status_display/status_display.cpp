// rqt plugin displaying the controller status
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "status_display.h"

#include <ros/node_handle.h>
#include <ros/service.h>

#include <QMessageBox>

#include <pluginlib/class_list_macros.h>

#include <apc_interface/DimLight.h>
#include <apc_interface/SuctionStrength.h>
#include <std_srvs/SetBool.h>

Q_DECLARE_METATYPE(apc_interface::ControllerStateConstPtr)

namespace apc_interface
{

StatusDisplay::StatusDisplay()
{
}

StatusDisplay::~StatusDisplay()
{
}

void StatusDisplay::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	m_w = new QWidget();

	m_ui.setupUi(m_w);

	ctx.addWidget(m_w);

	qRegisterMetaType<apc_interface::ControllerStateConstPtr>();
	connect(this, SIGNAL(controllerStateReceived(apc_interface::ControllerStateConstPtr)),
		SLOT(handleControllerState(apc_interface::ControllerStateConstPtr)),
		Qt::QueuedConnection
	);

	ros::NodeHandle nh = getPrivateNodeHandle();
	m_sub_state = nh.subscribe("/apc_interface/controller_state", 1, &StatusDisplay::controllerStateReceived, this);

	connect(m_ui.vacuumButton, SIGNAL(clicked(bool)), SLOT(toggleVacuum()));
	connect(m_ui.lightButton, SIGNAL(clicked(bool)), SLOT(toggleLight()));
	connect(m_ui.vacuumPowerButton, SIGNAL(clicked(bool)), SLOT(toggleVacuumPower()));
	connect(m_ui.strengthSlider, SIGNAL(valueChanged(int)), SLOT(updateSliderLabel(int)));
}

void StatusDisplay::shutdownPlugin()
{
	m_sub_state.shutdown();
}

void StatusDisplay::handleControllerState(const apc_interface::ControllerStateConstPtr& msg)
{
	QPalette bgOff = m_ui.vacuumLabel->palette();
	QPalette bgHalf = bgOff;
	QPalette bgOn = bgOff;

	bgOff.setColor(QPalette::Window, Qt::red);
	bgHalf.setColor(QPalette::Window, Qt::yellow);
	bgOn.setColor(QPalette::Window, Qt::green);

	m_ui.vacuumLabel->setText(
		msg->vacuum_active ? "ON" : "OFF"
	);
	m_ui.vacuumLabel->setPalette(
		msg->vacuum_active ? bgOn : bgOff
	);

	m_ui.lightLabel->setText(
		QString("%1").arg((int)msg->light_duty)
	);
	m_ui.airVelocityLabel->setText(
		QString("%1").arg(msg->air_velocity)
	);
	m_ui.airVelocityLowPassLabel->setText(
		QString("%1").arg(msg->air_velocity_low_pass)
	);
	m_ui.airVelocityLabel->setPalette(
		msg->object_well_attached ? bgOn : (msg->something_on_tip ? bgHalf : bgOff)
	);

	m_ui.vacuumPowerLabel->setText(
		msg->vacuum_power_active ? "ON" : "OFF"
	);

	m_lastState = msg;
}

void StatusDisplay::toggleVacuum()
{
	if(!m_lastState)
		return;

	std_srvs::SetBool srv;
	apc_interface::SuctionStrength  suction;

	auto on =  !m_lastState->vacuum_active;
	srv.request.data = on;
	suction.request.strength = (on) ?
		(m_ui.strengthSlider->value() / 100.0f) : 0.0f;

	if(!ros::service::call("/apc_interface/set_suction_strength", suction))
	{
		QMessageBox::critical(m_w, "Error", "Could not call suction service");
	}

	if(!ros::service::call("/apc_interface/switch_vacuum", srv))
	{
		QMessageBox::critical(m_w, "Error", "Could not call vacuum service");
	}
}

void StatusDisplay::toggleVacuumPower()
{
	if(!m_lastState)
		return;

	std_srvs::SetBool srv;

	auto on =  !m_lastState->vacuum_power_active;
	srv.request.data = on;

	if(!ros::service::call("/apc_interface/switch_vacuum_power", srv))
	{
		QMessageBox::critical(m_w, "Error", "Could not call vacuum_power service");
	}
}

void StatusDisplay::toggleLight()
{
	if(!m_lastState)
		return;

	apc_interface::DimLight srv;

	srv.request.duty = m_lastState->light_duty ? 0 : 255;

	if(!ros::service::call("/apc_interface/dim", srv))
	{
		QMessageBox::critical(m_w, "Error", "Could not call light service");
	}
}

void StatusDisplay::updateSliderLabel(int ticks)
{
	auto strength = (float)(ticks / 100.0f);
	m_ui.strengthLabel->setText(
		QString::number(strength)
	);
	if(m_lastState)
	{
		if(m_lastState->vacuum_active)
		{
			apc_interface::SuctionStrength s;
			s.request.strength = strength;
			ros::service::call(
				"/apc_interface/set_suction_strength", s);
		}
	}
}

}

PLUGINLIB_EXPORT_CLASS(apc_interface::StatusDisplay, rqt_gui_cpp::Plugin)
