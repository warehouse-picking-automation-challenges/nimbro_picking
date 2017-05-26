// Non-maximum suppression
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "nms.h"

#include <algorithm>

namespace accel
{

std::vector<int> nms(
	const NMSRectangle* data,
	std::size_t count,
	float max_iou,
	int max_boxes
)
{
	std::vector<int> ret;

	if(count == 0)
		return ret;

	// Get vector of indices
	std::vector<int> indices(count);
	std::iota(indices.begin(), indices.end(), 0);

	// Sort it with ascending score
	std::sort(indices.begin(), indices.end(), [&](int a, int b) {
		// The LUA version actually uses <= here, but STL expects a less-than comparison
		// -- otherwise, it will actually corrupt the list.
		return data[a].score < data[b].score;
	});

	std::vector<float> area(count);
	for(std::size_t i = 0; i < count; ++i)
	{
		const auto& box = data[i];
		area[i] = (box.x2 - box.x1 + 1.0f) * (box.y2 - box.y1 + 1.0f);
	}

	while((max_boxes < 0 || ret.size() < max_boxes) && !indices.empty())
	{
		// Take the last box
		int current = indices.back();
		ret.push_back(current);

		const auto& box = data[current];

		float boxArea = area[current];

		// Only keep indices to boxes which have low IoU with this box
		std::size_t writeIndex = 0;
		for(std::size_t j = 0; j < indices.size() - 1; ++j)
		{
			const auto& other = data[indices[j]];

			float otherArea = area[indices[j]];

			// Calculate intersection
			float x1 = std::max(box.x1, other.x1);
			float x2 = std::min(box.x2, other.x2);
			float y1 = std::max(box.y1, other.y1);
			float y2 = std::min(box.y2, other.y2);

			float w = std::max(0.0f, x2 - x1 + 1.0f);
			float h = std::max(0.0f, y2 - y1 + 1.0f);

			float a_intersection = w*h;
			float a_union = boxArea + otherArea - a_intersection;

			float iou = a_intersection / a_union;

			if(iou <= max_iou)
			{
				indices[writeIndex++] = indices[j];
			}
		}

		indices.resize(writeIndex);
	}

	return ret;
}

}
