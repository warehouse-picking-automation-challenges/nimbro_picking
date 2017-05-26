#include "servo_filter_model.h"


namespace rqt_servo_diagnostics
{

ServoSortFilterModel::ServoSortFilterModel(QObject* parent)
: QSortFilterProxyModel(parent)
{
	setDynamicSortFilter(true);
	setFilterCaseSensitivity(Qt::CaseInsensitive);
}
ServoSortFilterModel::~ServoSortFilterModel()
{
}

}