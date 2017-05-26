#include <apc_shelf_model/dimensions.h>

namespace apc_shelf_model
{
std::vector<float> getToteCoordinates()
{
	//Origin of the tote frame is in the middle in X-Y-direction and on the ground surface.
	std::vector<float> values;
	values.push_back(TOTE_X / 2);
	values.push_back(TOTE_Y / 2);
	values.push_back(TOTE_Z);
	return values;
}


std::vector<float> getBoxCoordinates(int row, int col)
{

   float max_X = - SHELF_SEPERATOR_X;
   float min_X = max_X - BOX_DEPTH;
   float min_Y(0), max_Y(0);

   min_Y = SHELF_WIDTH + SHELF_SEPERATOR_Y ; // start with subtracting SHELF_SEPERATOR_Y. This is done to nullify the addition of
   // SHELF_SEPERATOR_Y done at first step of the loop
   for (int t=0;t<=col;t++)
   {
      max_Y = min_Y - SHELF_SEPERATOR_Y;
      if ( t%2 == 0 ) // for outer boxes
      {
         min_Y = max_Y - OUTER_BOX_WIDTH;
      }
      else
      {
         min_Y = max_Y - INNER_BOX_WIDTH;
      }
   }

   float min_Z(0), max_Z(0);

   min_Z = SHELF_HEIGHT + SHELF_SEPERATOR_Z;
   // same like Y. But start with adding Y since we ll be adding subtracting  it in the loops
   for (int t=0;t<=row;t++)
   {
      max_Z = min_Z - SHELF_SEPERATOR_Z;
      if ( t%3 == 0 ) // first and last box are of same dimension and inner two are same
      {
         min_Z = max_Z - OUTER_BOX_HEIGHT;
      }
      else
      {
         min_Z = max_Z - INNER_BOX_HEIGHT;
      }
   }

   std::vector<float> result;
   result.push_back(min_X);
   result.push_back(max_X);
   result.push_back(min_Y);
   result.push_back(max_Y);
   result.push_back(min_Z);
   result.push_back(max_Z);

   return result;
}


float getBoxMaxZ(int row)
{
	float max_Z(SHELF_HEIGHT - ((row +1) * SHELF_SEPERATOR_Z));

	switch(row)
	{
		case 0: return max_Z;
		case 1: return max_Z - OUTER_BOX_HEIGHT;
		case 2: return max_Z - OUTER_BOX_HEIGHT - INNER_BOX_HEIGHT;
		case 3: return max_Z - OUTER_BOX_HEIGHT - 2*INNER_BOX_HEIGHT;

		default: throw std::runtime_error("shelf row is not between 0 and 3!"); return 0;
	};
}

float getBoxMinZ(int row)
{
	float max_Z = SHELF_HEIGHT - (row+2)*SHELF_SEPERATOR_Z;

	switch(row)
	{
		case 0: return max_Z - OUTER_BOX_HEIGHT;
		case 1: return max_Z - OUTER_BOX_HEIGHT - INNER_BOX_HEIGHT;
		case 2: return max_Z - OUTER_BOX_HEIGHT - 2*INNER_BOX_HEIGHT;
		case 3: return max_Z - 2*OUTER_BOX_HEIGHT - 2*INNER_BOX_HEIGHT;

		default: throw std::runtime_error("shelf row is not between 0 and 3!"); return 0;
	};
}

}//namespace
