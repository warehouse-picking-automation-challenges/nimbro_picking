// Delegate for keyframe vel/acc limits
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "limit_delegate.h"

namespace nimbro_keyframe_editor
{

LimitDelegate::LimitDelegate(QObject* parent)
 : QItemDelegate(parent)
{
}

LimitDelegate::~LimitDelegate()
{
}

void LimitDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	QStyleOptionViewItem modOpt = option;

	// Don't let the selection override the background color, which is used to
	// indicate whether the limit is active.
	QVariant brush = index.data(Qt::BackgroundRole);
	if(brush.isValid())
		modOpt.state &= ~QStyle::State_Selected;

	QItemDelegate::paint(painter, modOpt, index);
}

}
