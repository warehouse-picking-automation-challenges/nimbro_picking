// Small GUI to tune filter settings
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef FILL_TUNER_H
#define FILL_TUNER_H

#include <QWidget>

#include <opencv2/opencv.hpp>

#include <depth_filler/depth_filler.h>
#include <depth_filler/domain_transform.h>

#include "ui_fill_tuner.h"

class FillTuner : public QWidget
{
Q_OBJECT
public:
	FillTuner();
	virtual ~FillTuner();

	void load(const std::string& file);
public Q_SLOTS:
	void setDepthErosion(int erosion);

	void compute();
	void refactorize();
	void computeDT();
	void setColorDistance(int dist);
	void setNormalizeWithVariance(bool on);
	void setDistanceExponent(double exp);
	void setDistanceScale(double scale);
	void toggleDepth();
	void display();

	void setSigmaR(double val);
	void setSigmaS(double val);
	void setNumIterations(int iter);
private:
	Ui_FillTuner m_ui;

	cv::Mat_<float> m_rawDepth;
	cv::Mat_<float> m_depth;
	cv::Mat_<cv::Vec3b> m_rgb;
	cv::Mat_<float> m_gray;

	cv::Mat_<float> m_filledDepth;
	cv::Mat_<float> m_dtFilledDepth;

	depth_filler::DepthFiller m_filler;
	depth_filler::DomainTransformFiller m_dtFiller;

	bool m_displayDepth = true;
};

#endif
