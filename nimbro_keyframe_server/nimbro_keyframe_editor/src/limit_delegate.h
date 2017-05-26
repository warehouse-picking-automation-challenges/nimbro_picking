// Delegate for keyframe vel/acc limits
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef LIMIT_DELEGATE_H
#define LIMIT_DELEGATE_H

#include <QItemDelegate>

namespace nimbro_keyframe_editor
{

class LimitDelegate : public QItemDelegate
{
public:
	explicit LimitDelegate(QObject* parent = 0);
	virtual ~LimitDelegate();

	virtual void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
};

}

#endif
