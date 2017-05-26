// Fast edge-preserving recursive filter using domain transform
// based on: Domain Transform for Edge-Aware Image and Video Processing
// by Gastal et al., 2011.
// Homepage: http://www.inf.ufrgs.br/~eslgastal/DomainTransform/

#ifndef DEPTH_FILLER_DOMAIN_TRANSFORM_H
#define DEPTH_FILLER_DOMAIN_TRANSFORM_H

#include <opencv/cv.hpp>

#include <memory>

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#endif

namespace depth_filler
{

class DomainTransformFillerPrivate;
class DomainTransformFiller
{
public:
	DomainTransformFiller();
	~DomainTransformFiller();

	void setSigmaR(double val);
	void setSigmaS(double val);
	void setNumIterations(int iter);

	cv::Mat_<float> fillDepth(const cv::Mat_<float>& depth, const cv::Mat_<float>& cvGrayscale);

#ifdef HAVE_PCL
	template<typename PointT>
	void fillCloud(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT>& output, float fx, float fy, float cx, float cy);
#endif
private:
	std::unique_ptr<DomainTransformFillerPrivate> m_d;
};

#ifdef HAVE_PCL

template<typename PointT>
void DomainTransformFiller::fillCloud(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT>& output, float fx, float fy, float cx, float cy)
{
	if(!input.isOrganized())
	{
		fprintf(stderr, "depth_filler::fillCloud(): Input cloud is not organized\n");
		return;
	}

	cv::Mat_<float> depth(input.height, input.width);
	cv::Mat_<cv::Vec3b> rgb(input.height, input.width);

	int inputIdx = 0;
	for(unsigned int y = 0; y < input.height; ++y)
	{
		for(unsigned int x = 0; x < input.width; ++x)
		{
			const auto& point = input[inputIdx];

			depth(y,x) = point.z;
			rgb(y,x) = cv::Vec3b(point.b, point.g, point.r);

			inputIdx++;
		}
	}

	cv::Mat_<uint8_t> grayU;
	cv::cvtColor(rgb, grayU, cv::COLOR_BGR2GRAY);
	cv::Mat_<float> gray;
	grayU.convertTo(gray, CV_32FC1);

	cv::Mat_<float> filled = fillDepth(depth, gray);

	output = input;
	unsigned int outputIdx = 0;
	for(unsigned int y = 0; y < input.height; ++y)
	{
		for(unsigned int x = 0; x < input.width; ++x)
		{
			PointT& p = output[outputIdx];

			if(!std::isfinite(p.z))
			{
// 				if(filled(y,x) == 0 || !std::isfinite(filled(y,x)))
// 					throw std::logic_error("output is not dense");

				p.z = filled(y,x);
				p.x = p.z * (x - cx) / fx;
				p.y = p.z * (y - cy) / fy;
			}

			outputIdx++;
		}
	}
}

#endif

}

#endif
