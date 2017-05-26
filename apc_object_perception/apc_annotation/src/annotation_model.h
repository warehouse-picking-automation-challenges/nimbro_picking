// Qt Item model for annotation masks
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ANNOTATION_MODEL_H
#define ANNOTATION_MODEL_H

#include <QtCore/QAbstractListModel>

#include <opencv2/core/core.hpp>

struct AnnotationItem
{
	QString name;
	std::vector<cv::Point> polygon;
};

class AnnotationModel : public QAbstractListModel
{
public:
	AnnotationModel(QObject* parent = 0);
	virtual ~AnnotationModel();

	int rowCount(const QModelIndex & parent = QModelIndex()) const override;

	QVariant data(const QModelIndex & index, int role) const override;

	void addItem(const QString& name, const std::vector<cv::Point>& polygon);

	inline AnnotationItem& item(unsigned int row)
	{ return m_items[row]; }

	inline const AnnotationItem& item(unsigned int row) const
	{ return m_items[row]; }

	void clear();
	bool loadFrom(const QString& filename);

	void save(const QString& filename);
	void saveMasks(const QString& dir, int width, int height);

	void deleteItem(int index);
private:
	QList<AnnotationItem> m_items;
};

#endif
