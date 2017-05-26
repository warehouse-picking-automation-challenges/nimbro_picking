// Annotate captured data
// Author: Arul Selvam Periyasamy
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "apc_annotation.h"

#include "ui_apc_annotation.h"

#include <QtCore/QDebug>

APCAnnotation::APCAnnotation()
{
	m_ui = new Ui_APCAnnotation;
	m_ui->setupUi(this);

	m_ui->view->setModel(&m_model);
	m_ui->view->setUndoStack(&m_undo);
	m_ui->listView->setModel(&m_model);

	m_ui->view->setSelectionModel(m_ui->listView->selectionModel());

	connect(m_ui->openButton, SIGNAL(clicked(bool)), SLOT(openDirectory()));
	connect(m_ui->frameSelectBox, SIGNAL(currentIndexChanged(QString)), SLOT(openFrame(QString)));

	connect(m_ui->nextButton, SIGNAL(clicked(bool)), SLOT(nextFrame()));
	connect(m_ui->prevButton, SIGNAL(clicked(bool)), SLOT(prevFrame()));

	connect(m_ui->deleteButton, SIGNAL(clicked(bool)), SLOT(deleteCurrentPolygon()));

	QAction* openAction = new QAction(this);
	openAction->setShortcut(QKeySequence(QKeySequence::Open));
	connect(openAction, SIGNAL(triggered(bool)), SLOT(openDirectory()));
	addAction(openAction);

	QAction* nextAction = new QAction(this);
	nextAction->setShortcut(QKeySequence(Qt::Key_Right));
	connect(nextAction, SIGNAL(triggered(bool)), SLOT(nextFrame()));
	addAction(nextAction);

	QAction* prevAction = new QAction(this);
	prevAction->setShortcut(QKeySequence(Qt::Key_Left));
	connect(prevAction, SIGNAL(triggered(bool)), SLOT(prevFrame()));
	addAction(prevAction);

	QAction* deleteAction = new QAction(this);
	deleteAction->setShortcut(QKeySequence(Qt::Key_Delete));
	connect(deleteAction, SIGNAL(triggered(bool)), SLOT(deleteCurrentPolygon()));
	addAction(deleteAction);

	QAction* undoAction = m_undo.createUndoAction(this);
	undoAction->setShortcut(QKeySequence::Undo);
	addAction(undoAction);

	QAction* redoAction = m_undo.createRedoAction(this);
	redoAction->setShortcut(QKeySequence::Redo);
	addAction(redoAction);

	QAction* abortPolygonAction = new QAction("Abort current polygon", this);
	abortPolygonAction->setShortcut(QKeySequence(Qt::Key_Escape));
	connect(abortPolygonAction, SIGNAL(triggered(bool)),
	        m_ui->view, SLOT(abortCurrentPolygon()));
	addAction(abortPolygonAction);

	readSettings();
}

APCAnnotation::~APCAnnotation()
{
	delete m_ui;
}

void APCAnnotation::closeEvent(QCloseEvent* event)
{
	// Remember window size and position for the next time

	QSettings settings("AIS", "APCAnnotation");
	settings.setValue("geometry", saveGeometry());
	settings.setValue("path", m_dirPath);

	if(!m_currentFrame.isEmpty())
		saveCurrentFrame();

	QWidget::closeEvent(event);
}

void APCAnnotation::readSettings()
{
	QSettings settings("AIS", "APCAnnotation");

	restoreGeometry(settings.value("geometry").toByteArray());
	m_dirPath = settings.value("path").toString();
}

void APCAnnotation::openDirectory()
{
	/*
	 * Iterate through the sub directories and look for image named "rgb.png"
	 */
	QString dirName;

	dirName = QFileDialog::getExistingDirectory(this, "Select a directory to open", m_dirPath);

	if(dirName.isEmpty())
		return;

	m_undo.clear();
	m_model.clear();
	m_currentFrame.clear();

	m_dirPath = dirName;

	QDir dir(dirName);
	QDirIterator it(dirName, QDir::NoDotAndDotDot | QDir::Dirs, QDirIterator::Subdirectories);

	QStringList frames;

	while(it.hasNext())
	{
		QString subDir = it.next();

		QString imgFile = subDir + "/rgb.png";
		if(!QFile(imgFile).exists())
			continue;

		frames.append(dir.relativeFilePath(subDir));
	}

	frames.sort();

	m_ui->frameSelectBox->clear();
	m_ui->frameSelectBox->addItems(frames);
}

void APCAnnotation::openFrame(const QString& frame)
{
	if(!m_currentFrame.isEmpty())
		saveCurrentFrame();

	QString rgbFile = m_dirPath + "/" + frame + "/rgb.png";

	m_ui->view->setBackground(QPixmap(rgbFile));

	m_model.clear();
	m_model.loadFrom(m_dirPath + "/" + frame + "/polygons.yaml");

	m_currentFrame = frame;
}

void APCAnnotation::nextFrame()
{
	if(m_ui->frameSelectBox->count() == 0)
		return;

	int index = m_ui->frameSelectBox->currentIndex();

	if(index == m_ui->frameSelectBox->count()-1)
	{
		QMessageBox::information(this, "No more images", "You reached the end of the dataset.");
		return;
	}

	m_ui->frameSelectBox->setCurrentIndex(index+1);
}

void APCAnnotation::prevFrame()
{
	if(m_ui->frameSelectBox->count() == 0)
		return;

	int index = m_ui->frameSelectBox->currentIndex();

	if(index == 0)
	{
		QMessageBox::information(this, "No more images", "You reached the start of the dataset.");
		return;
	}

	m_ui->frameSelectBox->setCurrentIndex(index-1);
}

void APCAnnotation::saveCurrentFrame()
{
	m_model.save(m_dirPath + "/" + m_currentFrame + "/polygons.yaml");
	m_model.saveMasks(m_dirPath + "/" + m_currentFrame,
		m_ui->view->background().width(),
		m_ui->view->background().height()
	);

	QPixmap pixmap(m_ui->view->size());
	m_ui->view->render(&pixmap);
	pixmap.save(m_dirPath + "/" + m_currentFrame + "/annotation.png");
}

void APCAnnotation::deleteCurrentPolygon()
{
	QModelIndex idx = m_ui->listView->currentIndex();

	if(!idx.isValid())
		return;

	m_model.deleteItem(idx.row());
}
