// Annotation view / editor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ANNOTATION_VIEW_H
#define ANNOTATION_VIEW_H

#include "item_model.h"

#include <QtGui/QWidget>

#include <opencv2/core/core.hpp>

class AnnotationModel;
class QItemSelectionModel;
class QUndoStack;

class AnnotationView : public QWidget
{
Q_OBJECT
public:
	AnnotationView(QWidget* parent = 0);
	virtual ~AnnotationView();

	void setUndoStack(QUndoStack* undo);

	void setBackground(const QPixmap& pixmap);

	inline const QPixmap& background() const
	{ return m_background; }

	void setModel(AnnotationModel* model);
	void setSelectionModel(QItemSelectionModel* model);

	void addPoint(const cv::Point& point);
	void removeLastPoint();

public Q_SLOTS:
	void abortCurrentPolygon();

protected:
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;

	void paintEvent(QPaintEvent *) override;

private:
	void updateTransform();

	QPoint mapPoint(const cv::Point& point) const;

	QUndoStack* m_undo = 0;

	QPixmap m_background;
	AnnotationModel* m_model = 0;
	QItemSelectionModel* m_selectionModel = 0;

	std::vector<cv::Point> m_currentPolygon;

	ItemModel m_itemModel;

	QTransform m_panZoom;
	QTransform m_transform;

	QPoint m_lastPan;
};

#endif
