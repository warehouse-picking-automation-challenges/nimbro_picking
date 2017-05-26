
#ifndef CONNECTEDCOMPONENTS_H
#define CONNECTEDCOMPONENTS_H

#include <opencv2/imgproc/imgproc.hpp>

//! connected components algorithm output formats
enum ConnectedComponentsTypes {
    CC_STAT_LEFT   = 0, //!< The leftmost (x) coordinate which is the inclusive start of the bounding
                        //!< box in the horizontal direction.
    CC_STAT_TOP    = 1, //!< The topmost (y) coordinate which is the inclusive start of the bounding
                        //!< box in the vertical direction.
    CC_STAT_WIDTH  = 2, //!< The horizontal size of the bounding box
    CC_STAT_HEIGHT = 3, //!< The vertical size of the bounding box
    CC_STAT_AREA   = 4, //!< The total area (in pixels) of the connected component
    CC_STAT_MAX    = 5
};

int connectedComponentsWithStats(cv::InputArray _img, cv::OutputArray _labels, cv::OutputArray statsv,
                                     cv::OutputArray centroids, int connectivity, int ltype);

#endif
