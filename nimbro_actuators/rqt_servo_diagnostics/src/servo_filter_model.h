#ifndef SERVO_FILTER_MODEL_H
#define SERVO_FILTER_MODEL_H

#include <QSortFilterProxyModel>
#include <QString>
#include <QRegExp>
#include "servo_model.h"

namespace rqt_servo_diagnostics
{
class ServoSortFilterModel : public QSortFilterProxyModel
{
Q_OBJECT
public:
	ServoSortFilterModel(QObject* parent = 0);
	virtual ~ServoSortFilterModel();
private:
	QRegExp m_filterExp;
};
}
#endif