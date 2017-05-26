// Generate proposal bounding boxes from RGB-D data
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/integral_image_normal.h>

#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>

#include <random>
#include <chrono>

#include "proposal_generator.h"

namespace fs = boost::filesystem;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

namespace apc_densecap
{

class MyComparator : public pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>
{
public:
	bool compare(int idx1, int idx2) const override
	{
		auto& point1 = input_->points[idx1];
		auto& point2 = input_->points[idx2];

		if(!pcl::isFinite(point1) || !pcl::isFinite(point2))
			return false;

		float dx = input_->points[idx1].x - input_->points[idx2].x;
		float dy = input_->points[idx1].y - input_->points[idx2].y;
		float dz = input_->points[idx1].z - input_->points[idx2].z;
		float dist = sqrtf (dx*dx + dy*dy + dz*dz);

		auto norm1 = normals_->points[idx1].getNormalVector3fMap();
		auto norm2 = normals_->points[idx2].getNormalVector3fMap();
		double normalAngle = norm1.dot(norm2) / norm1.norm() / norm2.norm();

		if(dist > distance_threshold_)
			return false;

		if(std::isfinite(normalAngle) && normalAngle < angular_threshold_)
			return false;

// 		float dist2shelf1 = m_dist2Shelf(idx1 / m_dist2Shelf.cols, idx1 % m_dist2Shelf.cols);
// 		float dist2shelf2 = m_dist2Shelf(idx2 / m_dist2Shelf.cols, idx2 % m_dist2Shelf.cols);
//
// 		if(dist2shelf1 < m_shelfThreshold || dist2shelf2 < m_shelfThreshold)
// 			return false;

		cv::Mat_<cv::Vec3f> rgbIn(1, 2);
		rgbIn(0, 0) = cv::Vec3f(point1.b / 255.0f, point1.g / 255.0f, point1.r / 255.0f);
		rgbIn(0, 1) = cv::Vec3f(point2.b / 255.0f, point2.g / 255.0f, point2.r / 255.0f);

		cv::Mat_<cv::Vec3f> hsvOut(1,2);

		cv::cvtColor(rgbIn, hsvOut, cv::COLOR_BGR2HSV);

		bool saturated1 = hsvOut(0,0)[1] > 0.3;
		bool saturated2 = hsvOut(0,1)[1] > 0.3;
		if(saturated1 != saturated2)
			return false;

		if(std::abs(hsvOut(0,0)[2] - hsvOut(0,1)[2]) > 0.5)
			return false;

		if(saturated1 && saturated2)
		{
			float angleDiff = hsvOut(0,0)[0] - hsvOut(0,1)[0];
			while(angleDiff > 180)
				angleDiff -= 360;
			while(angleDiff < -180)
				angleDiff += 360;

			if(std::abs(angleDiff) > color_threshold_)
				return false;
		}

		return true;
	}

	void setColorThreshold(float t)
	{ color_threshold_ = t; }

	void setDist2Shelf(const cv::Mat_<float>& dist)
	{ m_dist2Shelf = dist; }

	void setShelfThreshold(float shelf)
	{ m_shelfThreshold = shelf; }
private:
	float color_threshold_ = 10;
	cv::Mat_<float> m_dist2Shelf;
	float m_shelfThreshold = 0.02;
};

cv::Mat_<float> readRawFile(const fs::path& path)
{
	FILE* f = fopen(path.c_str(), "r");

	if(!f)
	{
		std::stringstream ss;
		ss << "Could not open data file: " << path.string();
		throw std::runtime_error(ss.str());
	}

	cv::Mat_<float> ret(1080, 1920);

	for(int y = 0; y < ret.rows; ++y)
	{
		if((int)fread(ret[y], sizeof(float), ret.cols, f) != ret.cols)
			throw std::runtime_error("Short read");
	}

	fclose(f);

	return ret;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> generateProposals(const PointCloud::ConstPtr& input, const pcl::PointCloud<pcl::Normal>::ConstPtr& normalCloud, const cv::Mat_<uint8_t>& mask_box, /*const cv::Mat_<float>& dist2Shelf,*/
	const ProposalParameters& params, cv::Mat_<cv::Vec3b>* vis)
{
	// Segment
	pcl::PointCloud<pcl::Label> labels;
	std::vector<pcl::PointIndices> labelIndices;
// 	{
		pcl::Label initialLabel;
		initialLabel.label = 0;
		auto initialLabels = boost::make_shared<pcl::PointCloud<pcl::Label>>(input->width, input->height, initialLabel);

		std::vector<bool> excludeLabels(input->size(), false);

		auto comparator = boost::make_shared<MyComparator>();
		comparator->setInputCloud(input);
		comparator->setInputNormals(normalCloud);
		comparator->setLabels(initialLabels);
		comparator->setExcludeLabels(excludeLabels);

		comparator->setAngularThreshold(params.angularThreshold * M_PI / 180.0f);
		comparator->setDistanceThreshold(params.distanceThreshold, false);
		comparator->setColorThreshold(params.colorThreshold);
		comparator->setShelfThreshold(params.shelfThreshold);
// 		comparator->setDist2Shelf(dist2Shelf);

		pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGB, pcl::Label> segmentation(comparator);

		segmentation.setInputCloud(input);
		segmentation.segment(labels, labelIndices);
// 	}

	if(vis)
		*vis = cv::Mat_<cv::Vec3b>(input->height, input->width);

	std::mt19937 rd;
	std::uniform_int_distribution<int> distr(0, 255);

	std::vector<cv::Rect> rectangles;

	{
		if(vis)
			*vis = cv::Vec3b(0, 0, 0);

		for(auto& indices : labelIndices)
		{
// 			if(indices.indices.size() < 600)
// 				continue;

// 			auto& color = COLORS[cnt % COLORS.size()];

			int minX = -1;
			int minY = -1;
			int maxX = -1;
			int maxY = -1;

			int numPoints = 0;

			cv::Vec3b color;

			if(vis)
				color = cv::Vec3b(distr(rd), distr(rd), distr(rd));

			for(auto& index : indices.indices)
			{
				int y = index / input->width;
				int x = index % input->width;

				if(mask_box(y,x) == 0)
					continue;

				if(minX < 0)
				{
					minX = indices.indices[0] % input->width;
					minY = indices.indices[0] / input->width;
					maxX = minX;
					maxY = minY;
				}

				minX = std::min(minX, x);
				minY = std::min(minY, y);
				maxX = std::max(maxX, x);
				maxY = std::max(maxY, y);

				numPoints++;

				if(vis)
					(*vis)(y,x) = color;
			}

			if(numPoints >= 200)
				rectangles.push_back(cv::Rect(cv::Point(minX, minY), cv::Point(maxX, maxY)));
		}
	}

	std::sort(rectangles.begin(), rectangles.end(), [](const cv::Rect& a, const cv::Rect& b) {
		return a.area() > b.area();
	});

	Eigen::MatrixXf rectData(1000, 4);
	unsigned int numValid = 0;

	for(size_t i = 0; i < std::min<size_t>(rectangles.size(), 1000); ++i)
	{
		if(rectangles[i].area() > (int)(input->width*input->height/2))
			continue;

		if(rectangles[i].area() < 100*100)
			continue;

		float scale = 720.0 / 1920.0;

		rectData(numValid, 0) = scale * (1920 - 1 - (rectangles[i].x + rectangles[i].width/2));
		rectData(numValid, 1) = scale * (1080 - 1 - (rectangles[i].y + rectangles[i].height/2));
		rectData(numValid, 2) = scale * rectangles[i].width;
		rectData(numValid, 3) = scale * rectangles[i].height;

		numValid++;
	}

	return rectData.topRows(numValid);
}

void compute(const fs::path& path, const ProposalParameters& params)
{
	PointCloud::Ptr input(new PointCloud);
	pcl::io::loadPCDFile((path / "frame.pcd").string(), *input);

	if(!input->isOrganized())
		throw std::runtime_error("Input is not organized");

	cv::Mat_<cv::Vec3b> mask_box = cv::imread(
		(path / "mask_box.png").string(), CV_LOAD_IMAGE_GRAYSCALE
	);

	cv::Mat_<float> dist2shelf = readRawFile(path / "feature_dist2shelf.raw");

	auto start = std::chrono::high_resolution_clock::now();

	// Compute normals
	pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
	{
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

		ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor(0.8f);
		ne.setNormalSmoothingSize(80.0f);

		ne.setInputCloud(input);
		ne.compute(*normalCloud);
	}

	cv::Mat_<cv::Vec3b> vis;
	auto rectangles = generateProposals(input, normalCloud, mask_box, /*dist2shelf,*/ params, &vis);

	auto end = std::chrono::high_resolution_clock::now();
	printf("Proposal generator took %ld ms\n", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

	cv::Mat_<cv::Vec3b> rgb = cv::imread((path / "rgb.png").string());

	std::mt19937 rd;
	std::uniform_int_distribution<int> distr(0, 255);

	for(int i = 0; i < rectangles.rows(); ++i)
	{
		float invScale = 1920.0 / 720.0;

		Eigen::VectorXf row = invScale * rectangles.row(i);
		cv::Rect rect(
			1920 - 1 - row[0] - row[2]/2,
			1080 - 1 - row[1] - row[3]/2,
			row[2], row[3]
		);

		cv::rectangle(rgb, rect, cv::Scalar(distr(rd), distr(rd), distr(rd)), 3);
	}

	cv::imwrite((path / "proposals.png").string(), rgb);
	cv::imwrite((path / "proposal_vis.png").string(), vis);

	{
		static_assert(decltype(rectangles)::IsRowMajor, "need row major storage for saving");

		FILE* f = fopen((path / "rgb.png.prop.bbox.bin").c_str(), "w");
		fwrite(rectangles.data(), sizeof(float), rectangles.rows()*4, f);
		fclose(f);
	}
}

}

int main(int argc, char** argv)
{
	namespace po = boost::program_options;

	std::string pathStr = ".";
	apc_densecap::ProposalParameters params;

	po::options_description opt("Options");
	opt.add_options()
		("help", "Help message")
		("angular",
			po::value(&params.angularThreshold)->default_value(params.angularThreshold),
			"Angular threshold for normal direction"
		)
		("distance",
			po::value(&params.distanceThreshold)->default_value(params.distanceThreshold),
			"Distance threshold"
		)
		("color",
			po::value(&params.colorThreshold)->default_value(params.colorThreshold),
			"Color threshold"
		)
		("shelf",
			po::value(&params.shelfThreshold)->default_value(params.shelfThreshold),
			"Shelf distance threshold"
		)
		("path",
			po::value(&pathStr)->default_value(pathStr),
			"Path to operate on"
		)
	;

	po::positional_options_description positional;
	positional.add("path", 1);

	po::variables_map vm;
	po::store(
		po::command_line_parser(argc, argv).options(opt).positional(positional).run(),
		vm
	);
	po::notify(vm);

	if(vm.count("help"))
	{
		std::cout << opt << std::endl;
		return 0;
	}

	// Gather list of images
	std::vector<fs::path> jobs;
	{
		auto it = fs::recursive_directory_iterator(pathStr);
		auto end = fs::recursive_directory_iterator();

		for(; it != end; ++it)
		{
			fs::path path = *it;
			if(fs::is_directory(path) && fs::exists(path / "polygons.yaml"))
				jobs.push_back(fs::canonical(path));
		}
	}

	unsigned int finished = 0;

	printf("\n");

	#pragma omp parallel for
	for(size_t i = 0; i < jobs.size(); ++i)
	{
		apc_densecap::compute(jobs[i], params);

		#pragma omp critical (data)
		{
			finished++;
			printf("\r%03u/%03lu", finished, jobs.size());
			fflush(stdout);
		}
	}

	printf("\n");

	return 0;
}
