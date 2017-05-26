// Depth filling based on the paper
//  "Colorization Using Optimization" by A. Levin, D. Lischinski, Y. Weiss
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//  based on an implementation by Benedikt Waldvogel

#ifndef DEPTH_FILLER_DEPTH_FILLER_H
#define DEPTH_FILLER_DEPTH_FILLER_H

#include <opencv2/opencv.hpp>

#include <memory>

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#endif

namespace depth_filler
{

class DepthFillerPrivate;
class DepthFiller
{
public:
	DepthFiller();
	~DepthFiller();

	enum ColorDistance
	{
		CD_GRAYSCALE,
		CD_RGB
	};

	void setColorDistance(ColorDistance dist);
	void setNormalizeWithVariance(bool on);
	void setDistanceExponent(float exp);
	void setDistanceScale(float scale);

	cv::Mat_<float> prefill(const cv::Mat_<float>& input);
	cv::Mat fillDepth(const cv::Mat& input, const cv::Mat& rgb, bool sameStructure = false, const cv::Mat_<uint8_t>& mask = cv::Mat_<uint8_t>());

	void erodeDepth(const cv::Mat_<float>& input, cv::Mat_<float>& output, int kernelSize);

	void dumpSystem(const std::string& prefix);

#ifdef HAVE_PCL
	template<typename PointT>
	void fillCloud(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT>& output, float fx, float fy, float cx, float cy, unsigned int erode = 0);
#endif
private:
	std::unique_ptr<DepthFillerPrivate> m_d;
};

#ifdef HAVE_PCL

template<typename PointT>
void DepthFiller::fillCloud(const pcl::PointCloud<PointT>& input, pcl::PointCloud<PointT>& output, float fx, float fy, float cx, float cy, unsigned int erode)
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

	if(erode)
		erodeDepth(depth, depth, erode);

	cv::Mat_<float> filled = fillDepth(depth, rgb);

	output = input;
	unsigned int outputIdx = 0;
	for(unsigned int y = 0; y < input.height; ++y)
	{
		for(unsigned int x = 0; x < input.width; ++x)
		{
			PointT& p = output[outputIdx];

			if(!std::isfinite(p.z))
			{
				if(filled(y,x) == 0 || !std::isfinite(filled(y,x)))
					throw std::logic_error("output is not dense");

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
