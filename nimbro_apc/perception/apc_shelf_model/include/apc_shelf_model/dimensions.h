// Dimensions of the APC shelf
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef APC_SHELF_MODEL_DIMENSIONS_H
#define APC_SHELF_MODEL_DIMENSIONS_H

#include <ros/console.h>

namespace apc_shelf_model
{

/* The dimension of the boxes:
 *
 *	DEPTH corresponds to X axis
 *	WIDTH corresponds to Y axis
 *	***The boxes are not of same dimensions***
 *	The width of center box is larger than the outer boxes
 *	The outer boxes are of similar dimension
 */
constexpr float SHELF_HEIGHT     		= 1.820;
constexpr float SHELF_SEPERATOR_Z 		= 0.04;

constexpr float OUTER_BOX_HEIGHT		= 0.230;
constexpr float INNER_BOX_HEIGHT		= 0.190;


constexpr float SHELF_WIDTH     		= 0.425;
constexpr float SHELF_SEPERATOR_Y 		= 0.006;

constexpr float OUTER_BOX_WIDTH			= 0.274;
constexpr float INNER_BOX_WIDTH			= 0.298;


constexpr float BOX_DEPTH				= 0.440;
constexpr float SHELF_SEPERATOR_X		= 0.003;


constexpr float TOTE_X 					= 0.615;
constexpr float TOTE_Y					= 0.373;
constexpr float TOTE_Z					= 0.2;


std::vector<float> getBoxCoordinates(int row, int col);
std::vector<float> getToteCoordinates();


float getBoxMaxZ(int row);
float getBoxMinZ(int row);

}

#endif
