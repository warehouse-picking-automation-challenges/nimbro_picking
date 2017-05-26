// Rectangle representation (center + extent)
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "rectangle.h"

namespace apc_densecap
{

Rectangle findClosestRectangle(const std::vector<Rectangle>& rectangles, const Rectangle& key)
{
	double minDist = -1;
	Rectangle minRect;

	for(const Rectangle& rect : rectangles)
	{
		double dist = (key.cx - rect.cx)*(key.cx - rect.cx) + (key.cy - rect.cy)*(key.cy - rect.cy);

		if(minDist < 0 || dist < minDist)
		{
			minDist = dist;
			minRect = rect;
		}
	}

	return minRect;
}

}
