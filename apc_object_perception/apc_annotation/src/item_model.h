// Qt model for the APC objects
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ITEM_MODEL_H
#define ITEM_MODEL_H

#include <QtCore/QAbstractListModel>
#include <QtGui/QPixmap>

class ItemModel : public QAbstractListModel
{
public:
	ItemModel();
	virtual ~ItemModel();

	int rowCount(const QModelIndex & parent = QModelIndex()) const override;
	QVariant data(const QModelIndex & index, int role) const override;
private:
	std::vector<std::pair<std::string, QPixmap>> m_objects;
};

#endif
