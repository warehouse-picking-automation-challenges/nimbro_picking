// Servo diagnostics GUI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SERVO_DIAGNOSTICS_H
#define SERVO_DIAGNOSTICS_H

#include <rqt_gui_cpp/plugin.h>
#include <QCheckBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QVBoxLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>


#include <ros/subscriber.h>

#include "servo_model.h"
#include "servo_filter_model.h"

#if HAVE_DRC_NETWORK
#include <drc_network/ServoTemperature.h>
#endif

class QTableView;
class ServoSortFilterModel;

namespace rqt_servo_diagnostics
{

class ServoDiagnostics : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	ServoDiagnostics();
	virtual ~ServoDiagnostics();

	virtual void initPlugin(qt_gui_cpp::PluginContext& ctx) override;
	virtual void shutdownPlugin() override;

	virtual void saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const override;
	virtual void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings) override;
Q_SIGNALS:
	void dataReceived(const actuator_msgs::DiagnosticsConstPtr& msg);
#if HAVE_DRC_NETWORK
	void temperatureReceived(const drc_network::ServoTemperatureConstPtr& msg);
#endif

private Q_SLOTS:
	void showContextMenu(const QPoint& point);
	void handleRebootAction();
	void handleFadeAction();
	void beepSlot();
	void conditionalBeep();
	void fadeButtonAction();
	void filterChangedSlot();
private:
	ServoModel m_model;
	ServoSortFilterModel* m_proxy;

	ros::Subscriber m_sub_diag;
#if HAVE_DRC_NETWORK
	ros::Subscriber m_sub_fast_diag;
#endif

	QTableView* m_view;
	QVBoxLayout* m_layout_v;
	QHBoxLayout* m_beep_layout_h;
	QHBoxLayout* m_fade_layout_h;
	QCheckBox* m_beep_checkbox;
	QPushButton* m_beep_testbutton;
	QSpinBox* m_beep_temp_input;
	QSpinBox* m_beep_critical_temp_input;
	QTimer* m_beep_timer;
	QDoubleSpinBox* m_torque_input;
	QDoubleSpinBox* m_torque_time_input;
	QLabel* m_torque_input_label;
	QLabel* m_torque_time_input_label;
	QPushButton* m_fade_button;
	QLabel* m_servo_filter_label;
	QLineEdit* m_servo_filter_input;
	QWidget *q;

	uint64_t m_beep_counter;
};

}

#endif

