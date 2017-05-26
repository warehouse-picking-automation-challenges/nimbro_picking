// Small test driver for the apc_6dpose library
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_6dpose/apc_6dpose.h>

#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

#include <boost/make_shared.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <yaml-cpp/yaml.h>

//TEMP
#include <pcl/common/centroid.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <boost/any.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <boost/program_options.hpp>


namespace fs = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
    std::string input_file;
    std::string output_file;
    std::string class_name;
    std::string grasp_file;
    std::string models_dir = "/home/aura/Documents/Robotics/apc_objects/model_kleenex_paper_towels/";

    po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("input,i", po::value<std::string>(&input_file)->required(), "Input pcd to annotate")
        ("models_dir", po::value<std::string>(&models_dir), "Input pcd to annotate")
        ("output,o", po::value<std::string>(&output_file), "Input pcd to annotate")
        ("class_name,o", po::value<std::string>(&class_name)->required(), "Output file")
        ("grasp_file", po::value<std::string>(&grasp_file)->required(), "Initialization file")

   ;

    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    po::store(parsed, vm);
    if (vm.count("help")) { std::cout << desc << std::endl; }
    try { po::notify(vm); }
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }

    fs::path imgPath(input_file);

    auto cloud = boost::make_shared<apc_6dpose::APC6DPose::PointCloud>();
	if(pcl::io::loadPCDFile((imgPath / "frame.pcd").string(), *cloud) < 0)
	{
		fprintf(stderr, "Could not load PCD file\n");
		return 1;
	}

    apc_6dpose::APC6DPose registration;
    registration.setUseNaiveApproach(true);
    registration.setUseVisibility(false);
    registration.initialize(models_dir, grasp_file);

	// Load ground-truth mask
    cv::Mat_<uint8_t> mask = cv::imread((imgPath / ("mask_" + class_name + ".png")).string(), CV_LOAD_IMAGE_GRAYSCALE);


	// Find center point
	Eigen::Vector3f center = Eigen::Vector3f::Zero();
	unsigned int count = 0;

    for(size_t y = 0; y < cloud->height; ++y)
	{
        for(size_t x = 0; x < cloud->width; ++x)
		{
			auto p = (*cloud)(x,y);
			if(mask(y,x) && pcl::isFinite(p))
			{
				center += p.getVector3fMap();
				count++;
			}
		}
	}

	center /= count;

    auto result = registration.compute(cloud, class_name, center, mask);

    std::cout << "Resulting pose:\n" << result.objectPose.matrix() << "\n";
    std::cout << "Grasping position saved to "  << output_file << "," << result.grasping_points.points.size() << ", " << result.grasping_points.width << ", "<< result.grasping_points.height <<"graping_points.pcd"<< std::endl;

    pcl::io::savePCDFileASCII (output_file+ "graping_points.pcd", result.grasping_points);

	return 0;
}
