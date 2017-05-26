// Request item name from user
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "item_dialog.h"

#include "ui_item_dialog.h"

#include <QtCore/QTimer>

#include <QtGui/QSortFilterProxyModel>

ItemDialog::ItemDialog(QAbstractItemModel* model, QWidget* parent)
 : QDialog(parent)
{
	m_ui = new Ui_ItemDialog;
	m_ui->setupUi(this);

	m_completer.setModel(model);
	m_ui->searchEdit->setCompleter(&m_completer);

	QSortFilterProxyModel* proxy(new QSortFilterProxyModel(this));
	proxy->setSourceModel(model);

	connect(m_ui->searchEdit, SIGNAL(textChanged(QString)), proxy, SLOT(setFilterRegExp(QString)));

	m_ui->listView->setModel(proxy);

	connect(m_ui->okButton, SIGNAL(clicked(bool)), SLOT(accept()));
	connect(m_ui->cancelButton, SIGNAL(clicked(bool)), SLOT(reject()));

	connect(m_ui->listView->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)), SLOT(validate()));

	connect(m_ui->listView, SIGNAL(activated(QModelIndex)), SLOT(accept()));

	// Autofocus on search bar
	QTimer::singleShot(0, m_ui->searchEdit, SLOT(setFocus()));
}

ItemDialog::~ItemDialog()
{
	delete m_ui;
}

QString ItemDialog::name() const
{
	QModelIndex idx = m_ui->listView->currentIndex();

	return idx.data(Qt::DisplayRole).toString();
}

void ItemDialog::validate()
{
	m_ui->okButton->setEnabled(m_ui->listView->currentIndex().isValid());
}
