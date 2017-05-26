// Benchmark for nms module
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "nms.h"

#include <cstdio>
#include <fstream>

#include <chrono>

int main(int argc, char** argv)
{
	if(argc != 2)
	{
		fprintf(stderr, "Usage: nms_bench <boxes CSV>\n");
		return 1;
	}

	std::ifstream boxFile(argv[1]);

	std::vector<accel::NMSRectangle> boxes;
	for(std::string line; std::getline(boxFile, line); )
	{
		accel::NMSRectangle rect;
		sscanf(line.c_str(), "%f %f %f %f %f",
			&rect.x1, &rect.y1, &rect.x2, &rect.y2, &rect.score
		);

		boxes.push_back(rect);
	}

	std::chrono::milliseconds sumDur(0);

	for(int i = 0; i < 100; ++i)
	{
		auto start = std::chrono::high_resolution_clock::now();
		std::vector<int> indices = accel::nms(boxes.data(), boxes.size(), 0.7, 1000);
		auto end = std::chrono::high_resolution_clock::now();

		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
// 		fprintf(stderr, "NMS took %ld ms\n", ms.count());

		sumDur += ms;
	}

	fprintf(stderr, "Average: %f ms\n", float(sumDur.count()) / 100);

// 	for(int idx : indices)
// 		printf("%d\n", idx+1); // output LUA-style indices for comparison

	return 0;
}
