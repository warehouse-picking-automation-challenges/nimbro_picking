// Provides a list of buttons
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "buttonview.h"

#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>

Q_DECLARE_METATYPE(QModelIndex)

ButtonView::ButtonView(QWidget* parent)
 : QAbstractItemView(parent)
{
	QVBoxLayout* layout = new QVBoxLayout(viewport());
	layout->setSpacing(0);
	layout->setContentsMargins(0, 0, 0, 0);

	setBackgroundRole(QPalette::Window);
	viewport()->setBackgroundRole(QPalette::Background);
	setFrameShape(QFrame::NoFrame);
}

ButtonView::~ButtonView()
{
}

QPushButton* ButtonView::addButton(const QModelIndex& index)
{
	QString label = model()->data(index).toString();

	QPushButton* btn = new QPushButton(label, this);

	btn->setProperty("index", QVariant::fromValue(index));

	connect(btn, SIGNAL(clicked(bool)), SLOT(emitClick()));

	return btn;
}


void ButtonView::rowsInserted(const QModelIndex& parent, int start, int end)
{
	QAbstractItemView::rowsInserted(parent, start, end);

	QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(viewport()->layout());

	for(int i = start; i <= end; ++i)
	{
		QModelIndex idx = model()->index(i, 0);

		QPushButton* btn = addButton(idx);

		layout->insertWidget(i, btn);
		m_buttons.insert(i, btn);
	}
}

void ButtonView::rowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
	QAbstractItemView::rowsAboutToBeRemoved(parent, start, end);

	for(int i = end; i >= start; --i)
	{
		delete m_buttons[i];
		m_buttons.removeAt(i);
	}
}

void ButtonView::reset()
{
	QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(viewport()->layout());

	QAbstractItemView::reset();

	qDeleteAll(m_buttons);
	m_buttons.clear();

	if(!model())
		return;

	for(int i = 0; i < model()->rowCount(); ++i)
	{
		QModelIndex idx = model()->index(i, 0);

		QPushButton* btn = addButton(idx);

		layout->addWidget(btn);
		m_buttons << btn;
	}
}

int ButtonView::horizontalOffset() const
{
	return 0;
}

int ButtonView::verticalOffset() const
{
	return 0;
}

QModelIndex ButtonView::indexAt(const QPoint& point) const
{
	QWidget* child = childAt(point);
	QPushButton* button = qobject_cast<QPushButton*>(child);

	if(!button)
		return QModelIndex();

	return button->property("index").value<QModelIndex>();
}

bool ButtonView::isIndexHidden(const QModelIndex& index) const
{
	return false;
}

QModelIndex ButtonView::moveCursor(QAbstractItemView::CursorAction cursorAction, Qt::KeyboardModifiers modifiers)
{
	return QModelIndex();
}

void ButtonView::scrollTo(const QModelIndex& index, QAbstractItemView::ScrollHint hint)
{
}

void ButtonView::setSelection(const QRect& rect, QItemSelectionModel::SelectionFlags command)
{
}

QRect ButtonView::visualRect(const QModelIndex& index) const
{
	QList<QPushButton*> list = findChildren<QPushButton*>();
	foreach(QPushButton* btn, list)
	{
		if(btn->property("index").value<QModelIndex>() == index)
			return btn->geometry();
	}

	return QRect();
}

void ButtonView::dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight, const QVector< int >& roles)
{
	reset();
}

QRegion ButtonView::visualRegionForSelection(const QItemSelection& selection) const
{
	return QRegion();
}

void ButtonView::emitClick()
{
	buttonClicked(sender()->property("index").value<QModelIndex>());
}

int ButtonView::sizeHintForRow(int row) const
{
	return 10;
}

