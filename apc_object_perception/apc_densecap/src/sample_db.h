// Holds training samples for SVM training / testing
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SAMPLE_DB_H
#define SAMPLE_DB_H

#include <memory>
#include <string>
#include <vector>
#include <map>

#include <Eigen/Core>

#include <boost/filesystem.hpp>

#include "rectangle.h"
#include "parameters.h"

namespace apc_densecap
{

class SampleDB
{
public:
	struct TrainingFrame
	{
		typedef std::shared_ptr<TrainingFrame> Ptr;

		std::string name;

		std::vector<float> featureData;
		std::vector<float> hhaData;
		std::vector<float> bboxData;
		std::map<std::string, std::vector<Eigen::VectorXf>> samples;
	};

	void loadDatasetFiles(const Parameters& params, const std::vector<boost::filesystem::path>& files);

	void loadDataset(const boost::filesystem::path& path, const Parameters& params);

	const std::map<std::string, TrainingFrame::Ptr>& files() const
	{ return m_data; }

	// Static utility functions
	static std::map<std::string, std::vector<Rectangle>> loadRectangles(
		const boost::filesystem::path& yamlPath
	);

	static void loadRawFloats(
		std::vector<float>* dest, const boost::filesystem::path& path,
		bool adaptive = false
	);

	static TrainingFrame::Ptr loadImage(const boost::filesystem::path& path, const Parameters& params);
private:
	std::map<std::string, TrainingFrame::Ptr> m_data;
};

}

#endif
