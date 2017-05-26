// Method hyperparameters
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PARAMETERS_H
#define PARAMETERS_H

namespace apc_densecap
{

struct Parameters
{
	double min_iou = 0.3425768894440784;
	double C = 121.43178291973112;
	unsigned int num_top_rectangles = 2;
	double biasScale = 9.532687787998906;
	bool tailor = true;
	bool scale = false;

	bool train_request = false;
	bool test_request = false;

	bool train_proposals = true;
	bool test_proposals = true;

	bool visualize = false;

	bool hha = false;

	int subsample = 1;
};

}

#endif

