// Qt model for connected servos
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "servo_model.h"

#include <QColor>

namespace rqt_servo_diagnostics
{

ServoModel::ServoModel(QObject* parent)
 : QAbstractTableModel(parent)
{
}

ServoModel::~ServoModel()
{
}

int ServoModel::columnCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return COL_COUNT;
}

int ServoModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return m_joints.size();
}

QVariant ServoModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(orientation != Qt::Horizontal || role != Qt::DisplayRole)
		return QAbstractItemModel::headerData(section, orientation, role);

	switch(section)
	{
		case COL_NAME:                return "Name";
		case COL_TEMPERATURE:         return "Temp";
		case COL_LOAD:                return "Torque";
		case COL_TORQUELIMIT:         return "Torque limit";
		case COL_TIMEOUT_CONFIDENCE:  return "Timeout Conf";
		case COL_TIMEOUTS:            return "#Timeouts";
		case COL_ERROR:               return "Error";
		case COL_VOLTAGE:             return "Voltage";
		default:                      return QVariant();
	}
}

QVariant ServoModel::data(const QModelIndex& index, int role) const
{
	if(!index.isValid())
		return QVariant();

	if(index.row() < 0 || index.row() >= (int)m_joints.size())
		return QVariant();

	const auto& joint = m_joints[index.row()];

	switch(role)
	{
		case Qt::DisplayRole:
			switch(index.column())
			{
				case COL_NAME:
					return QString::fromStdString(joint.name);
				case COL_TEMPERATURE:
					return joint.temperature;
				case COL_LOAD:
					return fabs(joint.torque);
				case COL_TORQUELIMIT:
					return joint.goalTorque;
				case COL_TIMEOUT_CONFIDENCE:
					return joint.timeoutConfidence;
				case COL_TIMEOUTS:
					return joint.timeouts;
				case COL_ERROR:
					return QString::number(joint.error);
				case COL_VOLTAGE:
					return QString::number(joint.voltage/10.0);
			}
			break;
		case Qt::BackgroundColorRole:
			switch(index.column())
			{
// 				case COL_TEMPERATURE:
// 				{
// 					const float MIN_TEMP = 40;
// 					const float MAX_TEMP = 80;
// 					float temp = joint.temperature;
// 					float alpha = (temp - MIN_TEMP) / (MAX_TEMP - MIN_TEMP);
// 					alpha = std::min(1.0f, std::max(0.0f, alpha));
//
// 					return QColor(255, (1.0f - alpha) * 255, (1.0f - alpha) * 255);
// 				}
				case COL_ERROR:
				{
					if(joint.error)
						return QColor(Qt::red);
					else
						return QVariant();
				}
			}

			break;
		case Qt::TextAlignmentRole:
			switch(index.column())
			{
				case COL_TEMPERATURE:
				case COL_LOAD:
				case COL_TORQUELIMIT:
				case COL_ERROR:
				case COL_TIMEOUTS:
				case COL_TIMEOUT_CONFIDENCE:
					return (int)(Qt::AlignVCenter | Qt::AlignRight);
				default:
					return QVariant();
			}
			break;
	}

	return QVariant();
}

void ServoModel::updateData(const actuator_msgs::DiagnosticsConstPtr& msg)
{
	for(auto jointIn : msg->joints)
	{
		auto it = std::lower_bound(m_joints.begin(), m_joints.end(), jointIn, [&](const actuator_msgs::JointDiagnostics& A, const actuator_msgs::JointDiagnostics& B) {
			return A.name < B.name;
		});

		int row = it - m_joints.begin();

		if(it != m_joints.end() && it->name == jointIn.name)
		{
			// Data update
			*it = jointIn;
			dataChanged(index(row, 1), index(row, COL_COUNT-1));
		}
		else
		{
			// Row insert
			beginInsertRows(QModelIndex(), row, row);
			m_joints.insert(it, jointIn);
			endInsertRows();
		}
	}
}

#if HAVE_DRC_NETWORK
void ServoModel::updateTemperature(const drc_network::ServoTemperatureConstPtr& msg)
{
	m_joints[msg->index].temperature = msg->temperature;
}
#endif


QString ServoModel::jointName(int row) const
{
	if(row < 0 || row >= (int)m_joints.size())
		return QString();

	return QString::fromStdString(m_joints[row].name);
}

int ServoModel::overTemp(int beepTemp)
{
	for(auto JointDiagnosticsIt : m_joints)
	{
		if(JointDiagnosticsIt.temperature > beepTemp)
			return JointDiagnosticsIt.temperature;
	}
	return 0;
}



}
