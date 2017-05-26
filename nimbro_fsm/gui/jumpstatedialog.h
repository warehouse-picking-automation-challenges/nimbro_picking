// Select a state to jump to
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef JUMPSTATEDIALOG_H
#define JUMPSTATEDIALOG_H

#include <QtGui/QDialog>
#include <QtCore/QModelIndex>

class Ui_JumpStateDialog;
class QStringListModel;
class QSortFilterProxyModel;

namespace nimbro_fsm
{

class JumpStateDialog : public QDialog
{
Q_OBJECT
public:
	explicit JumpStateDialog(const QStringList& states, QWidget* parent = 0);
	virtual ~JumpStateDialog();

	inline QString selectedState() const
	{ return m_selectedState; }
private Q_SLOTS:
	void handleClick(const QModelIndex& idx);
private:
	Ui_JumpStateDialog* m_ui;
	QStringListModel* m_model;
	QSortFilterProxyModel* m_proxy;

	QString m_selectedState;
};

}

#endif
