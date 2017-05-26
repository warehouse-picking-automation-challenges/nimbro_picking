// Image widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "image_widget.h"

#include <QPainter>
#include <QDebug>

ImageWidget::ImageWidget(QWidget* parent)
 : QWidget(parent)
{
}

ImageWidget::~ImageWidget()
{
}

void ImageWidget::setRGB(const cv::Mat_<cv::Vec3b>& input)
{
	QImage img(input.data, input.cols, input.rows, QImage::Format_RGB888);
	m_pixmap = QPixmap::fromImage(img.rgbSwapped());
	update();
	updateGeometry();
}

void ImageWidget::setDepth(const cv::Mat_<float>& input)
{
	QImage img(input.cols, input.rows, QImage::Format_ARGB32);

	for(int y = 0; y < input.rows; ++y)
	{
		for(int x = 0; x < input.cols; ++x)
		{
			auto& depth = input(y,x);
			if(!std::isfinite(depth))
				img.setPixel(x,y, qRgba(0xFF, 0x00, 0x00, 0xFF));
			else
			{
				const float SCALE = 300;
				uint8_t gray = SCALE * depth;
				img.setPixel(x, y, qRgba(gray, gray, gray, 0xFF));
			}
		}
	}

	m_pixmap = QPixmap::fromImage(img);
	update();
	updateGeometry();
}

void ImageWidget::paintEvent(QPaintEvent*)
{
	if(m_pixmap.isNull())
		return;

	QRect imageRect = m_pixmap.rect();

	float scale = std::min(
		(float)width() / imageRect.width(),
		(float)height() / imageRect.height()
	);

	QRect scaled(0, 0, scale * imageRect.width(), scale * imageRect.height());

	QPainter painter(this);
	painter.drawPixmap(scaled, m_pixmap);
}

QSize ImageWidget::sizeHint() const
{
	return m_pixmap.size();
}

QSize ImageWidget::minimumSizeHint() const
{
	return m_pixmap.size();
}
