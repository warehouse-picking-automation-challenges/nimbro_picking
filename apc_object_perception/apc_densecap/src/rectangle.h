// Rectangle representation (center + extent)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef RECTANGLE_H
#define RECTANGLE_H

#include <algorithm>

#include <opencv2/core/core.hpp>

namespace apc_densecap
{

struct Rectangle
{
	Rectangle()
	 : cx(0), cy(0), w(0), h(0)
	{}

	Rectangle(double cx, double cy, double w, double h)
	 : cx(cx), cy(cy), w(w), h(h)
	{}

	double area() const
	{ return w*h; }

	double left() const
	{ return cx - 0.5*w; }

	double right() const
	{ return cx + 0.5*w; }

	double top() const
	{ return cy - 0.5*h; }

	double bottom() const
	{ return cy + 0.5*h; }

	cv::Point2d topLeft() const
	{ return cv::Point2d(left(), top()); }

	cv::Point2d topRight() const
	{ return cv::Point2d(right(), top()); }

	cv::Point2d bottomLeft() const
	{ return cv::Point2d(left(), bottom()); }

	cv::Point2d bottomRight() const
	{ return cv::Point2d(right(), bottom()); }

	Rectangle intersection(const Rectangle& other) const
	{
		double nleft = std::max(left(), other.left());
		double nright = std::min(right(), other.right());
		double ntop = std::max(top(), other.top());
		double nbottom = std::min(bottom(), other.bottom());

		if(nleft >= nright || ntop >= nbottom)
			return Rectangle(0, 0, 0, 0);

		return Rectangle(
			(nleft+nright)/2.0, (ntop+nbottom)/2.0,
			nright-nleft, nbottom-ntop
		);
	}

	double iou(const Rectangle& other) const
	{
		double intersectionArea = intersection(other).area();
		if(intersectionArea == 0)
			return 0;

		assert(intersectionArea >= 0);
		assert(intersectionArea <= area() + 1e-7);
		assert(intersectionArea <= other.area() + 1e-7);

		return intersectionArea / (area() + other.area() - intersectionArea);
	}

	Rectangle unionRect(const Rectangle& other) const
	{
		double nleft = std::min(left(), other.left());
		double nright = std::max(right(), other.right());
		double ntop = std::min(top(), other.top());
		double nbottom = std::max(bottom(), other.bottom());

		return Rectangle(
			(nleft+nright)/2.0, (ntop+nbottom)/2.0,
			nright-nleft, nbottom-ntop
		);
	}

	double cx;
	double cy;
	double w;
	double h;
};

Rectangle findClosestRectangle(const std::vector<Rectangle>& rectangles, const Rectangle& key);

template<class Iterator>
Iterator findClosestRectangleIt(Iterator begin, Iterator end, const Rectangle& key)
{
	double minDist = -1;
	Iterator minRect = end;

	for(Iterator it = begin; it != end; ++it)
	{
		const Rectangle& rect = *it;
		double dist = (key.cx - rect.cx)*(key.cx - rect.cx) + (key.cy - rect.cy)*(key.cy - rect.cy);

		if(minDist < 0 || dist < minDist)
		{
			minDist = dist;
			minRect = it;
		}
	}

	return minRect;
}

}

#endif
