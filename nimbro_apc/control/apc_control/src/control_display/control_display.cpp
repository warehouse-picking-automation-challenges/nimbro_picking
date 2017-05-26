// rqt plugin displaying the items inside the shelf and tote
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#include "control_display.h"
#include <boost/concept_check.hpp>
#include <ros/node_handle.h>
#include <ros/service.h>

#include <QMessageBox>
#include <std_srvs/Empty.h>

#include <pluginlib/class_list_macros.h>


Q_DECLARE_METATYPE(apc_control::ShelfStateConstPtr)

namespace apc_control
{

ControlDisplay::ControlDisplay()
{
}

ControlDisplay::~ControlDisplay()
{
}

void ControlDisplay::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	m_w = new QWidget();

	m_ui.setupUi(m_w);

	ctx.addWidget(m_w);

	m_timer = new QTimer();
	m_timer->setInterval(100);
	m_timer->start();

	qRegisterMetaType<apc_control::ShelfStateConstPtr>();
	connect(this, SIGNAL(shelfStateReceived(apc_control::ShelfStateConstPtr)),
		SLOT(handleShelfState(apc_control::ShelfStateConstPtr)),
		Qt::QueuedConnection
	);

	connect(m_timer, SIGNAL(timeout()), this, SLOT(timerTriggered()));
	connect(m_ui.start_button, SIGNAL(clicked()), this, SLOT(handleStart()));

	m_attempt_running = false;
	m_start_attempt = ros::Time::now();
	m_currentWorkItemIdx = 0;
	m_ui.start_button->setEnabled(false);

	ros::NodeHandle nh = getPrivateNodeHandle();
	m_sub_state = nh.subscribe("/apc_controller/shelf_state", 1, &ControlDisplay::shelfStateReceived, this);

}

void ControlDisplay::shutdownPlugin()
{
	m_sub_state.shutdown();
}


void ControlDisplay::handleShelfState(const apc_control::ShelfStateConstPtr& msg)
{


	m_start_attempt = msg->attemped_start_time;
	m_attempt_running = msg->attempt_running;
	m_currentWorkItemIdx = msg->currentWorkItem;
	m_start_item = msg->workingItems[msg->currentWorkItem].start_time;

	if(!m_attempt_running && m_start_attempt == ros::Time(0)){
		m_ui.label_time->setText("00:00:00");
		m_ui.start_button->setEnabled(true);
	}

	std::string title;
	if(msg->picking)
		title = "Picking";
	else
		title = "Stowing";

	title += ": " + msg->workingItems[m_currentWorkItemIdx].name;

	m_ui.label_title->setText(QString::fromStdString(title));


	std::string box;

	m_ui.list_shelf->clear();

	box = "--- A --- (" + std::to_string(msg->boxA.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxA)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- B --- (" + std::to_string(msg->boxB.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxB)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- C --- (" + std::to_string(msg->boxC.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxC)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- D --- (" + std::to_string(msg->boxD.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxD)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- E --- (" + std::to_string(msg->boxE.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxE)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- F --- (" + std::to_string(msg->boxF.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxF)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- G --- (" + std::to_string(msg->boxG.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxG)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- H --- (" + std::to_string(msg->boxH.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxH)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- I --- (" + std::to_string(msg->boxI.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxI)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- J --- (" + std::to_string(msg->boxJ.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxJ)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- K --- (" + std::to_string(msg->boxK.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxK)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- L --- (" + std::to_string(msg->boxL.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->boxL)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}

	box = "\n--- Tote --- (" + std::to_string(msg->tote.size()) + " items)";
	m_ui.list_shelf->addItem(QString::fromStdString(box));
	for(std::string item: msg->tote)
	{
		m_ui.list_shelf->addItem(QString::fromStdString(item));
	}


	m_ui.table_order->clear();
	m_ui.table_order->setRowCount(msg->workingItems.size());
	m_ui.table_order->setHorizontalHeaderItem(0, new QTableWidgetItem(QString("Item")));
	m_ui.table_order->setHorizontalHeaderItem(1, new QTableWidgetItem(QString("Order")));
	m_ui.table_order->setHorizontalHeaderItem(2, new QTableWidgetItem(QString("Points")));
	m_ui.table_order->setHorizontalHeaderItem(3, new QTableWidgetItem(QString("Time")));
// 	m_ui.table_order->horizontalHeader()->setResizeMode(QHeaderView::Stretch);


	int points = 0;

	for(unsigned int i=0;i<msg->workingItems.size();i++)
	{
		unsigned int pre_items=0;
		if(!msg->picking)
			pre_items=1;

		std::string text="";
		QString points_item("");
		if(i>pre_items)
		{
			text=msg->workingItems[i].location + " -> " + msg->workingItems[i].destination;
			if(msg->workingItems[i].success)
				points_item = QString::number(msg->workingItems[i].points);
			else
				points_item = "failed";
		}

		m_ui.table_order->setItem(i,0,new QTableWidgetItem(QString::fromStdString(msg->workingItems[i].name)));
		m_ui.table_order->setItem(i,1, new QTableWidgetItem(QString::fromStdString(text)));
		m_ui.table_order->setItem(i,2, new QTableWidgetItem(points_item));


		float item_duration = (msg->workingItems[i].end_time - msg->workingItems[i].start_time).toSec();

		if(item_duration > 0.0)
		{
			m_ui.table_order->setItem(i,3, new QTableWidgetItem(durationToString(msg->workingItems[i].end_time - msg->workingItems[i].start_time)));
		}
		else
		{
			m_ui.table_order->setItem(i,3, new QTableWidgetItem(QString("")));
		}

		if(msg->workingItems[i].success && msg->workingItems[i].end_time != ros::Time(0))
			points += msg->workingItems[i].points;

	}

	if(!m_attempt_running && m_currentWorkItemIdx > 0)
	{
		m_ui.table_order->setItem(m_currentWorkItemIdx, 3, new QTableWidgetItem(durationToString(ros::Time::now() - m_start_item)));
	}

	QString points_label(QString::number(points));
	points_label += QString(" Points");
	m_ui.label_points->setText(points_label);


}

void ControlDisplay::timerTriggered()
{

	if(!m_attempt_running)
		return;

	m_ui.label_time->setText(durationToString(ros::Time::now() - m_start_attempt));

	m_ui.table_order->setItem(m_currentWorkItemIdx, 3, new QTableWidgetItem(durationToString(ros::Time::now() - m_start_item)));

}


void ControlDisplay::handleStart()
{
	ros::NodeHandle nh = getPrivateNodeHandle();
	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/apc_controller/start");

	std_srvs::Empty srv;
	if (client.call(srv))
		ROS_INFO("started");
	else
		ROS_ERROR("Failed to call service start");
	m_ui.start_button->setEnabled(false);
}


QString ControlDisplay::durationToString(ros::Duration d)
{
	int run_seconds = d.toSec() * 10;
	int tenths = run_seconds % 10;
	int secs = run_seconds / 10;
	int mins = secs / 60;
	mins = mins % 60;
	secs = secs % 60;
	return (mins<10?"0":"") + QString::number(mins) + ":" + (secs<10?"0":"") + QString::number(secs) + ":" + QString::number(tenths);
}



}

PLUGINLIB_EXPORT_CLASS(apc_control::ControlDisplay, rqt_gui_cpp::Plugin)
