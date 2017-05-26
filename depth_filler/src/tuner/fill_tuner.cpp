// Small GUI to tune filter settings
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "fill_tuner.h"

#include <QApplication>
#include <QButtonGroup>
#include <QAction>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

FillTuner::FillTuner()
{
	m_ui.setupUi(this);

	connect(m_ui.computeBtn, SIGNAL(clicked(bool)), SLOT(compute()));
	connect(m_ui.refactorizeBtn, SIGNAL(clicked(bool)), SLOT(refactorize()));

	QButtonGroup* group = new QButtonGroup(this);
	group->addButton(m_ui.grayscaleButton, depth_filler::DepthFiller::CD_GRAYSCALE);
	group->addButton(m_ui.rgbButton, depth_filler::DepthFiller::CD_RGB);

	connect(group, SIGNAL(buttonClicked(int)), SLOT(setColorDistance(int)));

	connect(m_ui.normVarianceButton, SIGNAL(clicked(bool)), SLOT(setNormalizeWithVariance(bool)));

	connect(m_ui.distExponentBox, SIGNAL(valueChanged(double)), SLOT(setDistanceExponent(double)));
	connect(m_ui.distScaleBox, SIGNAL(valueChanged(double)), SLOT(setDistanceScale(double)));

	QAction* action = new QAction(this);
	action->setShortcut(QKeySequence(Qt::Key_Space));

	connect(action, SIGNAL(triggered()), SLOT(toggleDepth()));

	addAction(action);

	connect(m_ui.sigmaSBox, SIGNAL(valueChanged(double)), SLOT(setSigmaS(double)));
	connect(m_ui.sigmaRBox, SIGNAL(valueChanged(double)), SLOT(setSigmaR(double)));
	connect(m_ui.numIterBox, SIGNAL(valueChanged(int)), SLOT(setNumIterations(int)));

	connect(m_ui.erodeBox, SIGNAL(valueChanged(int)), SLOT(setDepthErosion(int)));
}

FillTuner::~FillTuner()
{
}

void FillTuner::setDepthErosion(int erode)
{
	if(erode != 0)
		m_filler.erodeDepth(m_rawDepth, m_depth, erode);
	else
		m_rawDepth.copyTo(m_depth);

	compute();
	computeDT();
}

void FillTuner::load(const std::string& file)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::io::loadPCDFile(file, cloud);

	if(cloud.size() == 0 || !cloud.isOrganized())
		throw std::runtime_error("Could not load cloud (or not organized)");

	m_rawDepth = cv::Mat_<float>(cloud.height, cloud.width);
	m_depth = cv::Mat_<float>(cloud.height, cloud.width);
	m_rgb = cv::Mat_<cv::Vec3b>(cloud.height, cloud.width);

	int inputIdx = 0;
	for(unsigned int y = 0; y < cloud.height; ++y)
	{
		for(unsigned int x = 0; x < cloud.width; ++x)
		{
			const auto& point = cloud[inputIdx];

			m_rawDepth(y,x) = point.z;
			m_rgb(y,x) = cv::Vec3b(point.b, point.g, point.r);

			inputIdx++;
		}
	}

	cv::Mat_<uint8_t> tmp;
	cv::cvtColor(m_rgb, tmp, cv::COLOR_BGR2GRAY);
	tmp.convertTo(m_gray, CV_32FC1);

	m_rawDepth.copyTo(m_depth);

	m_ui.rgb->setRGB(m_rgb);
	m_ui.raw->setDepth(m_depth);

// 	m_filledDepth = m_filler.fillDepth(m_depth, m_rgb);
	m_dtFilledDepth = m_dtFiller.fillDepth(m_depth, m_gray);
	computeDT();
	display();
}

void FillTuner::compute()
{
// 	m_filledDepth = m_filler.fillDepth(m_depth, m_rgb);
	display();
}

void FillTuner::refactorize()
{
// 	m_filledDepth = m_filler.fillDepth(m_depth, m_rgb, true);
	display();
}

void FillTuner::setColorDistance(int dist)
{
	if(dist < 0)
		return;

	m_filler.setColorDistance((depth_filler::DepthFiller::ColorDistance)dist);
	refactorize();
}

void FillTuner::setNormalizeWithVariance(bool on)
{
	m_filler.setNormalizeWithVariance(on);
	refactorize();
}

void FillTuner::setDistanceExponent(double exp)
{
	m_filler.setDistanceExponent(exp);
	refactorize();
}

void FillTuner::setDistanceScale(double scale)
{
	m_filler.setDistanceScale(scale);
	refactorize();
}

void FillTuner::toggleDepth()
{
	m_displayDepth = !m_displayDepth;
	display();
}

void FillTuner::computeDT()
{
	m_dtFilledDepth = m_dtFiller.fillDepth(m_depth, m_gray);
	display();
}

void FillTuner::display()
{
	if(m_displayDepth)
	{
		m_ui.filled->setDepth(m_filledDepth);
		m_ui.dtFilled->setDepth(m_dtFilledDepth);
		m_ui.raw->setDepth(m_depth);
	}
	else
	{
		m_ui.filled->setRGB(m_rgb);
		m_ui.dtFilled->setRGB(m_rgb);
		m_ui.raw->setRGB(m_rgb);
	}
}

void FillTuner::setSigmaR(double val)
{
	m_dtFiller.setSigmaR(val);
	computeDT();
}

void FillTuner::setSigmaS(double val)
{
	m_dtFiller.setSigmaS(val);
	computeDT();
}

void FillTuner::setNumIterations(int iter)
{
	m_dtFiller.setNumIterations(iter);
	computeDT();
}

int main(int argc, char** argv)
{
	QApplication app(argc, argv);

	if(app.arguments().size() < 2)
	{
		fprintf(stderr, "Usage: fill_tuner <pcd file>\n");
		return 1;
	}

	FillTuner win;
	win.load(app.arguments()[1].toStdString());
	win.show();

	app.exec();

	return 0;
}
