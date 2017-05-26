// Non-maximum suppression
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ACCEL_NMS_H
#define ACCEL_NMS_H

#include <vector>
#include <cstdlib>

// based on the DenseCap source code

namespace accel
{
	struct NMSRectangle
	{
		float x1;
		float y1;
		float x2;
		float y2;
		float score;
	};

	/**
	 * @brief Perform bounding box non-maximum suppression
	 *
	 * @param data Input boxes. Modified during the algorithm!
	 * @param count Number of input boxes
	 * @param max_iou Maximum IoU overlap between boxes in result
	 * @param max_boxes Maximum number of boxes (-1 = unlimited)
	 * @return List of surviving indices (in score-descending order)
	 **/
	std::vector<int> nms(
		const NMSRectangle* data,
		std::size_t count,
		float max_iou,
		int max_boxes = -1
	);
}

#endif
