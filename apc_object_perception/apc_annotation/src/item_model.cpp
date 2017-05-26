// Qt model for the APC objects
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "item_model.h"

#include <apc_objects/apc_objects.h>

#include <QtGui/QPainter>

ItemModel::ItemModel()
{
	m_objects.reserve(apc_objects::objects().size());

	const int GRID_SIZE = 120;

	for(const auto& obj : apc_objects::objects())
	{
		QPixmap raw(QString::fromStdString(obj.second.imagePath()));

		float scale = std::min(
			float(GRID_SIZE)/raw.width(),
			float(GRID_SIZE)/raw.height()
		);

		QRect rect(0, 0, scale * raw.width(), scale*raw.height());
		rect.translate(
			(GRID_SIZE - rect.width())/2,
			(GRID_SIZE - rect.height())/2
		);

		QImage img(GRID_SIZE, GRID_SIZE, QImage::Format_ARGB32);
		img.fill(qRgba(0,0,0,0));

		QPainter painter(&img);
		painter.drawPixmap(rect, raw);

		m_objects.emplace_back(obj.first, QPixmap::fromImage(img));
	}
}

ItemModel::~ItemModel()
{
}

int ItemModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return m_objects.size();
}

QVariant ItemModel::data(const QModelIndex& index, int role) const
{
	if(!index.isValid() || index.row() < 0 || index.row() >= rowCount())
		return QVariant();

	const auto& obj = m_objects[index.row()];

	switch(role)
	{
		case Qt::DisplayRole:
			return QString::fromStdString(obj.first);
		case Qt::DecorationRole:
			return obj.second;
		default:
			return QVariant();
	}
}
