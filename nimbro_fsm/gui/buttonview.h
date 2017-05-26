// Provides a list of buttons
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef BUTTONVIEW_H
#define BUTTONVIEW_H

#include <QtGui/QAbstractItemView>

class QPushButton;

class ButtonView : public QAbstractItemView
{
Q_OBJECT
public:
	explicit ButtonView(QWidget* parent = 0);
	virtual ~ButtonView();

	virtual int horizontalOffset() const;
	virtual int verticalOffset() const;

	virtual QRect visualRect(const QModelIndex& index) const;
	virtual QRegion visualRegionForSelection(const QItemSelection& selection) const;

	virtual QModelIndex indexAt(const QPoint& point) const;
	virtual bool isIndexHidden(const QModelIndex& index) const;

	virtual QModelIndex moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers);
	virtual void scrollTo(const QModelIndex& index, ScrollHint hint = EnsureVisible);
	virtual void setSelection(const QRect& rect, QItemSelectionModel::SelectionFlags command);

	virtual int sizeHintForRow(int row) const;
signals:
	void buttonClicked(const QModelIndex& index);
protected:
	virtual void dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight, const QVector< int >& roles = QVector<int>());
	void reset();
	virtual void rowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
	virtual void rowsInserted(const QModelIndex& parent, int start, int end);
private slots:
	void emitClick();
private:
	QPushButton* addButton(const QModelIndex& index);

	QList<QPushButton*> m_buttons;
};

#endif
