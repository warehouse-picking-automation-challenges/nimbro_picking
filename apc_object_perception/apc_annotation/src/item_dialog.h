// Request item name from user
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ITEM_DIALOG_H
#define ITEM_DIALOG_H

#include <QtGui/QDialog>
#include <QtGui/QCompleter>

class Ui_ItemDialog;

class ItemDialog : public QDialog
{
Q_OBJECT
public:
	ItemDialog(QAbstractItemModel* itemModel, QWidget* parent = 0);
	virtual ~ItemDialog();

	QString name() const;
private Q_SLOTS:
	void validate();
private:
	Ui_ItemDialog* m_ui;

	QCompleter m_completer;
};

#endif
