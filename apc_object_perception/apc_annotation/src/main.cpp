#include "apc_annotation.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	APCAnnotation w;
	w.show();

	return app.exec();
}
