// Image widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef IMAGE_WIDGET_H
#define IMAGE_WIDGET_H

#include <QWidget>

#include <opencv2/opencv.hpp>

class ImageWidget : public QWidget
{
public:
	ImageWidget(QWidget* parent = 0);
	virtual ~ImageWidget();

	void setRGB(const cv::Mat_<cv::Vec3b>& input);
	void setDepth(const cv::Mat_<float>& input);

	void paintEvent(QPaintEvent* event) override;
	QSize sizeHint() const override;
	QSize minimumSizeHint() const override;
private:
	QPixmap m_pixmap;
};

#endif
