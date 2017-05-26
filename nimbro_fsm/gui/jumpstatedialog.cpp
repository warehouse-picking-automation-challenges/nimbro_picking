// Select a state to jump to
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "jumpstatedialog.h"
#include "buttonview.h"

#include "ui_jumpstatedialog.h"

#include <QtGui/QStringListModel>
#include <QtGui/QSortFilterProxyModel>

namespace nimbro_fsm
{

JumpStateDialog::JumpStateDialog(const QStringList& states, QWidget* parent)
 : QDialog(parent)
{
	m_ui = new Ui_JumpStateDialog;
	m_ui->setupUi(this);

	m_model = new QStringListModel(states, this);

	m_proxy = new QSortFilterProxyModel(this);
	m_proxy->setSourceModel(m_model);
	m_proxy->setFilterCaseSensitivity(Qt::CaseInsensitive);

	m_ui->buttonView->setModel(m_proxy);

	QObject::connect(
		m_ui->filterEdit, SIGNAL(textEdited(QString)),
		m_proxy, SLOT(setFilterWildcard(QString))
	);
	QObject::connect(
		m_ui->buttonView, SIGNAL(buttonClicked(QModelIndex)),
		SLOT(handleClick(QModelIndex))
	);
}

JumpStateDialog::~JumpStateDialog()
{
	delete m_ui;
}

void JumpStateDialog::handleClick(const QModelIndex& idx)
{
	QModelIndex sourceIdx = m_proxy->mapToSource(idx);
	m_selectedState = m_model->data(sourceIdx, Qt::DisplayRole).toString();
	accept();
}

}
