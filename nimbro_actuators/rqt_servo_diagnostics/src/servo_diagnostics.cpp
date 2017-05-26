// Servo diagnostics GUI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "servo_diagnostics.h"

#include "bar_delegate.h"

#include <QTableView>
#include <QMenu>
#include <QMessageBox>
#include <QSortFilterProxyModel>
#include <QHeaderView>
#include <QTimer>
#include <QRegExp>

#include <pluginlib/class_list_macros.h>

#include <actuator_msgs/RebootServo.h>
#include <actuator_msgs/FadeJoints.h>

#include <ros/node_handle.h>
#include <ros/service.h>

#include <stdlib.h>


Q_DECLARE_METATYPE(actuator_msgs::DiagnosticsConstPtr)

#if HAVE_DRC_NETWORK
Q_DECLARE_METATYPE(drc_network::ServoTemperatureConstPtr)
#endif

namespace rqt_servo_diagnostics
{

ServoDiagnostics::ServoDiagnostics():
  m_beep_counter(0)
{
}

ServoDiagnostics::~ServoDiagnostics()
{
}

void ServoDiagnostics::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	qRegisterMetaType<actuator_msgs::DiagnosticsConstPtr>();
	connect(this, SIGNAL(dataReceived(actuator_msgs::DiagnosticsConstPtr)),
		&m_model, SLOT(updateData(actuator_msgs::DiagnosticsConstPtr))
	);

	m_sub_diag = getPrivateNodeHandle().subscribe(
		"/momaro/diagnostics", 1, &ServoDiagnostics::dataReceived, this
	);

#if HAVE_DRC_NETWORK
	connect(this, SIGNAL(temperatureReceived(drc_network::ServoTemperatureConstPtr)),
		&m_model, SLOT(updateTemperature(drc_network::ServoTemperatureConstPtr))
	);

	m_sub_fast_diag = getPrivateNodeHandle().subscribe(
		"/remote/servo_temp", 1, &ServoDiagnostics::temperatureReceived, this
	);
#endif

	m_proxy = new ServoSortFilterModel(this);
	m_proxy->setSourceModel(&m_model);
	m_proxy->setDynamicSortFilter(true);

	m_view = new QTableView;
	m_view->setModel(m_proxy);
	m_view->setSortingEnabled(true);
	m_view->verticalHeader()->hide();

	m_view->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(m_view, SIGNAL(customContextMenuRequested(QPoint)),
		SLOT(showContextMenu(QPoint))
	);

	m_view->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_view->setSelectionMode(QAbstractItemView::ExtendedSelection);

	BarDelegate* tempDelegate = new BarDelegate(m_view);
	tempDelegate->setRange(40.0, 80.0);
	m_view->setItemDelegateForColumn(ServoModel::COL_TEMPERATURE, tempDelegate);

	BarDelegate* torqueDelegate = new BarDelegate(m_view);
	torqueDelegate->setRange(0.0, 5.0);
	m_view->setItemDelegateForColumn(ServoModel::COL_LOAD, torqueDelegate);

	BarDelegate* timeoutConfDelegate = new BarDelegate(m_view);
	timeoutConfDelegate->setRange(0.0, 1.0);
	m_view->setItemDelegateForColumn(ServoModel::COL_TIMEOUT_CONFIDENCE, timeoutConfDelegate);

	connect(this, SIGNAL(dataReceived(actuator_msgs::DiagnosticsConstPtr)),
		m_view, SLOT(resizeRowsToContents())
	);
	
	m_beep_checkbox = new QCheckBox("Beep for high servo temperature");
	m_beep_checkbox->setCheckState(Qt::Checked);
	m_beep_testbutton = new QPushButton("Test beep");
	connect(m_beep_testbutton, SIGNAL(clicked()), this, SLOT(beepSlot()));

	m_beep_temp_input = new QSpinBox;
	m_beep_temp_input->setRange(30, 100);
	m_beep_temp_input->setSingleStep(1);
	m_beep_temp_input->setValue(71);

	m_beep_critical_temp_input = new QSpinBox;
	m_beep_critical_temp_input->setRange(30, 100);
	m_beep_critical_temp_input->setSingleStep(1);
	m_beep_critical_temp_input->setValue(75);
	
	m_beep_layout_h = new QHBoxLayout;
	m_beep_layout_h->addWidget(m_beep_checkbox);
	m_beep_layout_h->addWidget(m_beep_temp_input);
	m_beep_layout_h->addWidget(m_beep_critical_temp_input);
	m_beep_layout_h->addWidget(m_beep_testbutton);
	
	m_torque_input = new QDoubleSpinBox;
	m_torque_input->setRange(0.0, 1.0);
	m_torque_input->setValue(0.5);
	m_torque_input->setDecimals(4);
	m_torque_input_label = new QLabel("% of maxTorque in");
	m_torque_time_input = new QDoubleSpinBox;
	m_torque_time_input->setRange(0.5, 15.0);
	m_torque_time_input->setValue(1.5);
	m_torque_time_input->setDecimals(1);
	m_torque_time_input_label = new QLabel("seconds fade duration");
	m_fade_button = new QPushButton("Fade selected servos");
	connect(m_fade_button, SIGNAL(clicked()), this, SLOT(fadeButtonAction()));
	m_fade_layout_h = new QHBoxLayout;
	m_fade_layout_h->addWidget(m_torque_input);
	m_fade_layout_h->addWidget(m_torque_input_label);
	m_fade_layout_h->addWidget(m_torque_time_input);
	m_fade_layout_h->addWidget(m_torque_time_input_label);
	m_fade_layout_h->addWidget(m_fade_button);

	m_servo_filter_label = new QLabel;
	m_servo_filter_label->setText("Filter Servos (QRegExp):");
	m_servo_filter_input = new QLineEdit;
	connect(m_servo_filter_input, SIGNAL(textChanged(QString)), this, SLOT(filterChangedSlot()));
	
	m_layout_v = new QVBoxLayout;
	m_layout_v->addWidget(m_servo_filter_input);
	m_layout_v->addWidget(m_view);
	m_layout_v->addLayout(m_beep_layout_h);
	m_layout_v->addLayout(m_fade_layout_h);
	
	m_beep_timer = new QTimer(this);
	connect(m_beep_timer, SIGNAL(timeout()), this, SLOT(conditionalBeep()));
	m_beep_timer->setInterval(500);
	m_beep_timer->start();
	
	q = new QWidget;
	q->setLayout(m_layout_v);
	q->setWindowTitle("Servo Diagnostics");
	ctx.addWidget(q);
}

void ServoDiagnostics::beepSlot()
{
	if(system("beep") != 0)
		;
}

void ServoDiagnostics::conditionalBeep()
{
	int overTemp = m_model.overTemp(m_beep_temp_input->value());
	if(m_beep_checkbox->isChecked() && overTemp)
	{
		if(overTemp > m_beep_critical_temp_input->value())
			beepSlot();
		else if((m_beep_counter++ % 6) == 0)
			beepSlot();
	}
}

void ServoDiagnostics::shutdownPlugin()
{
	m_sub_diag.shutdown();

#if HAVE_DRC_NETWORK
	m_sub_fast_diag.shutdown();
#endif
}

void ServoDiagnostics::showContextMenu(const QPoint& point)
{
	QModelIndex idx = m_view->indexAt(point);

	idx = m_proxy->mapToSource(idx);

	if(!idx.isValid())
		return;

	QString jointName = m_model.jointName(idx.row());

	QMenu menu(m_view);


	QAction* action = menu.addAction("Reboot");
	QAction* fadeoutAction = menu.addAction("Fade Out");
	QAction* fadeinAction = menu.addAction("Fade in");

	action->setProperty("joint", jointName);
	fadeoutAction->setProperty("torque_enable", false);
	fadeinAction->setProperty("torque_enable", true);


	QStringList jointNames;
	QItemSelectionModel* sel = m_view->selectionModel();
	for(int i = 0; i < sel->selectedRows().size(); ++i)
	{
		QModelIndex in = sel->selectedRows()[i];
		jointNames << m_model.jointName(m_proxy->mapToSource(in).row());
	}

	fadeinAction->setProperty("jointNames", jointNames);
	fadeoutAction->setProperty("jointNames", jointNames);


	connect(action, SIGNAL(triggered()), SLOT(handleRebootAction()));
	connect(fadeoutAction, SIGNAL(triggered()), SLOT(handleFadeAction()));
	connect(fadeinAction, SIGNAL(triggered()), SLOT(handleFadeAction()));


	menu.exec(m_view->viewport()->mapToGlobal(point));
}

void ServoDiagnostics::handleFadeAction()
{
	QAction* action = qobject_cast<QAction*>(sender());
	if(!action)
		return;

	QStringList jointNames = action->property("jointNames").toStringList();
	bool torque_enable = action->property("torque_enable").toBool();

	actuator_msgs::FadeJoints srv;
	QString msg = QString("Please confirm that I fade %1 servos: ").arg(torque_enable ? "in" : "out");
	for(int i = 0; i < jointNames.size(); ++i)
	{
		srv.request.joints.push_back(jointNames[i].toStdString());
		msg.append("\n'" + jointNames[i] + "'");
	}

	srv.request.torque_proportion = torque_enable ? 1.0f : 0.0f;
	srv.request.fading_duration = ros::Duration(0.5);

	int btn = QMessageBox::question(m_view, "Confirm fading",
		msg, QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes
	);

	if(btn != QMessageBox::Yes)
		return;

	if(ros::service::call("/momaro/fade_joints", srv))
		QMessageBox::information(m_view, "Success", "Servo faded");
	else
		QMessageBox::critical(m_view, "Failure", "Could not send fade request");
}

void ServoDiagnostics::handleRebootAction()
{
	QAction* action = qobject_cast<QAction*>(sender());

	if(!action)
		return;

	QString name = action->property("joint").toString();

	int btn = QMessageBox::question(m_view, "Confirm reboot",
		QString("Please confirm that I should reboot servo '%1'").arg(name),
		QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes
	);

	if(btn != QMessageBox::Yes)
		return;

	actuator_msgs::RebootServo srv;
	srv.request.name = name.toStdString();

	if(ros::service::call("/momaro/reboot_servo", srv))
		QMessageBox::information(m_view, "Success", "Servo rebooted");
	else
		QMessageBox::critical(m_view, "Failure", "Could not send reboot request");
}

void ServoDiagnostics::restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings)
{
	if(instanceSettings.contains("tableViewState"))
		m_view->horizontalHeader()->restoreState(instanceSettings.value("tableViewState").toByteArray());
}

void ServoDiagnostics::saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const
{
	instanceSettings.setValue("tableViewState", m_view->horizontalHeader()->saveState());
}

void ServoDiagnostics::fadeButtonAction()
{
	QStringList jointNames;
	QItemSelectionModel* sel = m_view->selectionModel();
	for(int i = 0; i < sel->selectedRows().size(); ++i)
	{
		QModelIndex in = sel->selectedRows()[i];
		jointNames << m_model.jointName(m_proxy->mapToSource(in).row());
	}
	if(jointNames.isEmpty())
		return;
	actuator_msgs::FadeJoints srv;
	QString msg = QString("Please confirm that I the following %1 servos to %2 % power?").arg(jointNames.length()).arg(100*m_torque_input->value());
	for(int i = 0; i < jointNames.size(); ++i)
	{
		srv.request.joints.push_back(jointNames[i].toStdString());
		msg.append("\n'" + jointNames[i] + "'");
	}

	srv.request.torque_proportion = m_torque_input->value();
	srv.request.fading_duration = ros::Duration(m_torque_time_input->value());

	int btn = QMessageBox::question(m_view, "Confirm fading",
		msg, QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes
	);

	if(btn != QMessageBox::Yes)
		return;

	if(ros::service::call("/momaro/fade_joints", srv))
		QMessageBox::information(m_view, "Success", "Servo faded");
	else
		QMessageBox::critical(m_view, "Failure", "Could not send fade request");
}

void ServoDiagnostics::filterChangedSlot()
{
	m_proxy->setFilterRegExp(QRegExp(m_servo_filter_input->text()));
}


}

PLUGINLIB_EXPORT_CLASS(rqt_servo_diagnostics::ServoDiagnostics, rqt_gui_cpp::Plugin)
