// Annotation view / editor
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "annotation_view.h"
#include "annotation_model.h"
#include "item_dialog.h"

#include <QtGui/QPainter>
#include <QtGui/QMouseEvent>
#include <QtGui/QInputDialog>
#include <QtGui/QItemSelectionModel>
#include <QtGui/QUndoStack>

#include <QtCore/QDebug>

namespace undo
{
	class AddPointCommand : public QUndoCommand
	{
	public:
		AddPointCommand(AnnotationView* view, const cv::Point& point)
		 : QUndoCommand("add point")
		 , m_view(view)
		 , m_point(point)
		{
		}

		virtual void redo() override
		{
			m_view->addPoint(m_point);
		}

		virtual void undo() override
		{
			m_view->removeLastPoint();
		}
	private:
		AnnotationView* m_view;
		cv::Point m_point;
	};
}

AnnotationView::AnnotationView(QWidget* parent)
 : QWidget(parent)
{
}

AnnotationView::~AnnotationView()
{
}

void AnnotationView::setUndoStack(QUndoStack* undo)
{
	m_undo = undo;
}

void AnnotationView::setBackground(const QPixmap& pixmap)
{
	m_background = pixmap;
	m_panZoom.reset();
	m_currentPolygon.clear();
	m_undo->clear();

	updateTransform();
	update();
}

void AnnotationView::setModel(AnnotationModel* model)
{
	m_model = model;

	connect(m_model, SIGNAL(modelReset()), SLOT(update()));
	connect(m_model, SIGNAL(rowsInserted(QModelIndex,int,int)), SLOT(update()));
	connect(m_model, SIGNAL(rowsRemoved(QModelIndex,int,int)), SLOT(update()));
}

void AnnotationView::setSelectionModel(QItemSelectionModel* model)
{
	m_selectionModel = model;

	connect(m_selectionModel, SIGNAL(currentChanged(QModelIndex,QModelIndex)), SLOT(update()));
}

void AnnotationView::updateTransform()
{
	QRect pixmapRect = m_background.rect();

	float scale = std::min(
		(float)width()/pixmapRect.width(),
		(float)height()/pixmapRect.height()
	);

	// Apply pan/zoom initially
	m_transform = m_panZoom;

	// Rotate everything by 180°
	m_transform.translate(width()/2, height()/2);
	m_transform.rotate(180);
	m_transform.translate(-width()/2, -height()/2);

	// move pixmap to center
	m_transform.translate(
		(width() - scale * pixmapRect.width())/2,
		(height() - scale * pixmapRect.height())/2
	);

	// scale s.t. pixmap fits
	m_transform.scale(scale, scale);
}

QPoint AnnotationView::mapPoint(const cv::Point& point) const
{
	return QPoint(point.x, point.y);
}

void AnnotationView::paintEvent(QPaintEvent*)
{
	QPainter painter(this);

	updateTransform(); // FIXME: move to resizeEvent

	painter.setTransform(m_transform);
	painter.drawPixmap(m_background.rect(), m_background);

	for(int i = 0; i < m_model->rowCount(); ++i)
	{
		const AnnotationItem& item = m_model->item(i);
		QModelIndex index = m_model->index(i, 0);

		QPolygon polygon;
		for(auto& point : item.polygon)
			polygon.append(mapPoint(point));


		bool current = m_selectionModel && (m_selectionModel->currentIndex() == index);

		if(item.name != "ground_metal" && item.name != "side_bar" && item.name != "front_bar" && item.name != "box")
		{
			QPen pen;
			pen.setCosmetic(true);
			pen.setColor(current ? Qt::green : Qt::red);
			pen.setWidth(3);
			painter.setPen(pen);

			if(item.name != "box")
			{
				QBrush brush;
				brush.setColor(current ? QColor(0, 255, 0, 80) : QColor(255, 0, 0, 80));
				brush.setStyle(Qt::BDiagPattern);
				painter.setBrush(brush);
			}
			else
			{
				painter.setBrush(QBrush());
			}

			painter.drawPolygon(polygon);
		}
	}

	// Draw current polygon
	if(!m_currentPolygon.empty())
	{
		QPen pen;
		pen.setCosmetic(true);
		pen.setColor(Qt::yellow);
		pen.setWidth(3);
		painter.setPen(pen);

		QPoint last = mapPoint(m_currentPolygon[0]);
		painter.drawPoint(last);

		pen.setWidth(3);
		painter.setPen(pen);

		for(unsigned int i = 1; i < m_currentPolygon.size(); ++i)
		{
			QPoint next = mapPoint(m_currentPolygon[i]);
			painter.drawLine(last, next);
			last = next;
		}
	}
}

void AnnotationView::mousePressEvent(QMouseEvent* event)
{
	m_lastPan = event->pos();

	event->accept();
}

void AnnotationView::mouseMoveEvent(QMouseEvent* event)
{
	if(event->buttons() & Qt::RightButton)
	{
		QPoint delta = event->pos() - m_lastPan;

		m_panZoom = m_panZoom * QTransform::fromTranslate(delta.x(), delta.y());
		updateTransform();
		update();
	}

	m_lastPan = event->pos();
}

void AnnotationView::addPoint(const cv::Point& point)
{
	m_currentPolygon.push_back(point);
	update();
}

void AnnotationView::removeLastPoint()
{
	m_currentPolygon.pop_back();
	update();
}

void AnnotationView::mouseReleaseEvent(QMouseEvent* event)
{
	if(event->button() != Qt::LeftButton)
		return;

	QPoint mousePos = event->pos();

	mousePos = m_transform.inverted().map(mousePos);

	// If outside of pixmap, clamp
	if(mousePos.x() < 0)
		mousePos.setX(0);
	if(mousePos.y() < 0)
		mousePos.setY(0);
	if(mousePos.x() >= m_background.width())
		mousePos.setX(m_background.width()-1);
	if(mousePos.y() >= m_background.height())
		mousePos.setY(m_background.height()-1);

	cv::Point cvPoint(mousePos.x(), mousePos.y());

	if(!m_currentPolygon.empty() && cv::norm(m_currentPolygon.front() - cvPoint) < 10)
	{
		ItemDialog dlg(&m_itemModel, this);

		if(dlg.exec() == QDialog::Accepted)
		{
			m_model->addItem(dlg.name(), m_currentPolygon);
		}

		m_currentPolygon.clear();

		// FIXME: Undo across adding/removing items?
		m_undo->clear();
	}
	else
	{
		m_undo->push(new undo::AddPointCommand(this, cvPoint));
	}

	update();

	event->accept();
}

void AnnotationView::wheelEvent(QWheelEvent* event)
{
	float scale = std::pow(1.1f, float(event->delta()) / 120);

	QTransform zoom;

	zoom.translate(event->pos().x(), event->pos().y());
	zoom.scale(scale, scale);
	zoom.translate(-event->pos().x(), -event->pos().y());

	m_panZoom = m_panZoom * zoom;

	updateTransform();
	update();
}

void AnnotationView::abortCurrentPolygon()
{
	m_currentPolygon.clear();
	m_undo->clear();
	update();
}
