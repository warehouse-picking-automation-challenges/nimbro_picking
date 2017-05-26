// Qt model for connected servos
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SERVO_MODEL_H
#define SERVO_MODEL_H

#include <QAbstractTableModel>

#include <actuator_msgs/Diagnostics.h>

#if HAVE_DRC_NETWORK
#include <drc_network/ServoTemperature.h>
#endif

namespace rqt_servo_diagnostics
{

class ServoModel : public QAbstractTableModel
{
Q_OBJECT
public:
	enum Column
	{
		COL_NAME,
		COL_TEMPERATURE,
		COL_LOAD,
		COL_TORQUELIMIT,
		COL_ERROR,
		COL_TIMEOUT_CONFIDENCE,
		COL_TIMEOUTS,
		COL_VOLTAGE,

		COL_COUNT
	};

	explicit ServoModel(QObject* parent = 0);
	virtual ~ServoModel();

	virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

	virtual int rowCount(const QModelIndex & parent) const override;
	virtual int columnCount(const QModelIndex & parent) const override;
	virtual QVariant data(const QModelIndex & index, int role) const override;

	QString jointName(int row) const;
	int overTemp(int beepTemp);
public Q_SLOTS:
	void updateData(const actuator_msgs::DiagnosticsConstPtr& msg);

#if HAVE_DRC_NETWORK
	void updateTemperature(const drc_network::ServoTemperatureConstPtr& msg);
#endif
private:
	std::vector<actuator_msgs::JointDiagnostics> m_joints;
};

}

#endif
