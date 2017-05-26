// Annotate captured data
// Author: Arul Selvam Periyasamy <arulselvam@uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef APC_ANNOTATION_H
#define APC_ANNOTATION_H

#include "annotation_model.h"
#include "annotation_view.h"

#include <QtGui>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/package.h>

#include <iostream>
#include <string>
#include <vector>
#include <memory>

class Ui_APCAnnotation;

class APCAnnotation :  public QWidget
{
Q_OBJECT
public:
	APCAnnotation();
	~APCAnnotation();

private Q_SLOTS:
	void openDirectory();
	void openFrame(const QString& image);

	void nextFrame();
	void prevFrame();

	void deleteCurrentPolygon();

protected:
	void closeEvent(QCloseEvent *) override;

private:
	void readSettings();
	void saveCurrentFrame();

	QUndoStack m_undo;

	QString m_currentFrame;
	AnnotationModel m_model;

	Ui_APCAnnotation* m_ui;

	QString m_dirPath;
};

#endif // APC_ANNOTATION_H
