// Small executable to fill a PCD file
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <depth_filler/depth_filler.h>
#include <depth_filler/domain_transform.h>

#include <boost/program_options.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace po = boost::program_options;

void imagesToPointCloud(const cv::Mat_<float>& depth, const cv::Mat_<cv::Vec3b>& rgb,
		float fx, float fy, float cx, float cy,
		pcl::PointCloud<pcl::PointXYZRGB>& output)
{
	output.resize(rgb.rows*rgb.cols);
	output.width = rgb.cols;
	output.height = rgb.rows;

	unsigned int outputIdx = 0;
	for(unsigned int y = 0; y < output.height; ++y)
	{
		for(unsigned int x = 0; x < output.width; ++x)
		{
			pcl::PointXYZRGB& p = output[outputIdx];

			p.r = rgb(y,x)[2];
			p.g = rgb(y,x)[1];
			p.b = rgb(y,x)[0];

			p.z = depth(y,x);
			p.x = p.z * (x - cx) / fx;
			p.y = p.z * (y - cy) / fy;

			outputIdx++;
		}
	}
}

struct Blob
{
	std::vector<cv::Point2i> pixels;
	cv::Rect rect;

	bool operator<(const Blob& other) const
	{ return pixels.size() > other.pixels.size(); }
};

void FindBlobs(const cv::Mat &binary, std::vector < Blob > &blobs)
{
    blobs.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32SC1);

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < label_image.rows; y++) {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) {
            if(row[x] != 1) {
                continue;
            }

            Blob blob;
            cv::floodFill(label_image, cv::Point(x,y), label_count, &blob.rect, 0, 0, 8);

            for(int i=blob.rect.y; i < (blob.rect.y+blob.rect.height); i++) {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=blob.rect.x; j < (blob.rect.x+blob.rect.width); j++) {
                    if(row2[j] != label_count) {
                        continue;
                    }

                    blob.pixels.push_back(cv::Point2i(j,i));
                }
            }

            blobs.push_back(blob);

            label_count++;
        }
    }
}

int main(int argc, char** argv)
{
	float fx = NAN;
	float fy = NAN;
	float cx = -1;
	float cy = -1;
	unsigned int erode = 0;
	unsigned int subsample = 1;

	po::options_description desc("Options");
	desc.add_options()
		("help", "produce help message")
		("fx", po::value<float>(), "set focal length (x)")
		("fy", po::value<float>(), "set focal length (y)")
		("cx", po::value<float>(), "set optical center (x)")
		("cy", po::value<float>(), "set optical center (y)")
		("benchmark", "Run 20 times for profiling")
		("erode", po::value<unsigned int>(), "erode depth with specified kernel half-size")
		("subsample", po::value<unsigned int>(), "subsample input")
		("dump", po::value<std::string>(), "dump sparse system under prefix")
		("dump-prefill", po::value<std::string>(), "dump prefill cloud")
		("dump-input", po::value<std::string>(), "dump (subsampled) input cloud")
		("method", po::value<std::string>()->default_value("domain"), "method (domain or opt)")
	;

	po::options_description hidden("Hidden");
	hidden.add_options()
		("input-file", po::value<std::string>(), "input file")
		("output-file", po::value<std::string>(), "output file")
	;

	po::options_description cmdline;
	cmdline.add(desc).add(hidden);

	po::positional_options_description p;
	p.add("input-file", 1);
	p.add("output-file", 1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(cmdline).positional(p).run(), vm);
	po::notify(vm);

	if(vm.count("help"))
	{
		std::cout << desc << "\n";
		return 1;
	}

	if(!vm.count("input-file") || !vm.count("output-file"))
	{
		fprintf(stderr, "Require input and output file!\n");
		std::cerr << desc << "\n";
		return 1;
	}

	pcl::PointCloud<pcl::PointXYZRGB> input;
	pcl::io::loadPCDFile(vm["input-file"].as<std::string>(), input);

	if(input.size() == 0)
	{
		fprintf(stderr, "Could not read input file\n");
		return 1;
	}

	if(!input.isOrganized())
	{
		fprintf(stderr, "Error: Input is not organized!\n");
		return 1;
	}

	if(vm.count("fx") && vm.count("fy") && vm.count("cx") && vm.count("cy"))
	{
		fx = vm["fx"].as<float>();
		fy = vm["fy"].as<float>();
		cx = vm["cx"].as<float>();
		cy = vm["cy"].as<float>();
	}
	else
	{
		// Estimate from PCD...
		cx = input.width/2 + 0.5f;
		cy = input.height/2 + 0.5f;
		fx = 0.0f;
		fy = 0.0f;
		unsigned int count = 0;

		unsigned int idx = 0;
		for(int y = 0; y < (int)input.height; ++y)
		{
			for(int x = 0; x < (int)input.width; ++x)
			{
				const auto& point = input[idx++];

				if(!std::isfinite(point.z))
					continue;

				if((x - cx) * (y - cy) * point.z != 0)
				{
					fx += point.z / point.x * (x - cx);
					fy += point.z / point.y * (y - cy);
					count++;
				}
			}
		}

		fx /= count;
		fy /= count;

		printf("Calculated focal lengths: fx=%f, fy=%f\n", fx, fy);
	}

	if(vm.count("subsample"))
	{
		subsample = vm["subsample"].as<unsigned int>();
		fx /= subsample;
		fy /= subsample;
		cx = (cx - 0.5f) / subsample + 0.5f;
		cy = (cy - 0.5f) / subsample + 0.5f;
	}

	cv::Mat_<float> depth(input.height/subsample, input.width/subsample);
	depth = NAN;

	cv::Mat_<cv::Vec3b> rgb(input.height, input.width);

	int inputIdx = 0;
	for(unsigned int y = 0; y < input.height; ++y)
	{
		for(unsigned int x = 0; x < input.width; ++x)
		{
			const auto& point = input[inputIdx];

			auto dx = x / subsample;
			auto dy = y / subsample;

			if(std::isnan(depth(dy,dx)) || depth(dy,dx) > point.z)
				depth(dy,dx) = point.z;

			rgb(y,x) = cv::Vec3b(point.b, point.g, point.r);

			inputIdx++;
		}
	}

	cv::Mat_<cv::Vec3b> rgbOut;
	cv::resize(rgb, rgbOut, cv::Size(rgb.cols / subsample, rgb.rows / subsample), fx, fy, cv::INTER_AREA);

	pcl::PointCloud<pcl::PointXYZRGB> output;

	if(vm.count("dump-input"))
	{
		imagesToPointCloud(depth, rgbOut, fx, fy, cx, cy, output);
		if(pcl::io::savePCDFileBinary(vm["dump-input"].as<std::string>(), output) < 0)
		{
			fprintf(stderr, "Could not write prefill file\n");
		}
	}

	depth_filler::DepthFiller filler;

	// Fill in holes in depth data under very strict conditions
	cv::Mat_<float> tmp = filler.prefill(depth);
	depth = tmp;

	if(vm.count("erode"))
	{
		erode = vm["erode"].as<unsigned int>();
		filler.erodeDepth(depth, depth, erode);
	}

	cv::Mat_<float> filled;

	if(vm["method"].as<std::string>() == "opt")
	{
		if(!vm.count("benchmark"))
			filled = filler.fillDepth(depth, rgbOut);
		else
		{
			for(int i = 0; i < 20; ++i)
				filled = filler.fillDepth(depth, rgbOut);
		}
	}
	else if(vm["method"].as<std::string>() == "domain")
	{
		cv::Mat_<uint8_t> grayU;
		cv::cvtColor(rgbOut, grayU, cv::COLOR_BGR2GRAY);
		cv::Mat_<float> gray;
		grayU.convertTo(gray, CV_32FC1);

		depth_filler::DomainTransformFiller dmFiller;

		if(!vm.count("benchmark"))
			filled = dmFiller.fillDepth(depth, gray);
		else
		{
			for(int i = 0; i < 600; ++i)
				filled = dmFiller.fillDepth(depth, gray);
		}
	}
	else
		throw std::runtime_error("Unknown method specified");

	imagesToPointCloud(filled, rgbOut, fx, fy, cx, cy, output);
	if(pcl::io::savePCDFileBinary(vm["output-file"].as<std::string>(), output) < 0)
	{
		fprintf(stderr, "Could not write output file\n");
	}

	if(vm.count("dump-prefill"))
	{
		imagesToPointCloud(depth, rgbOut, fx, fy, cx, cy, output);
		if(pcl::io::savePCDFileBinary(vm["dump-prefill"].as<std::string>(), output) < 0)
		{
			fprintf(stderr, "Could not write prefill file\n");
		}
	}

	if(vm.count("dump"))
	{
		filler.dumpSystem(vm["dump"].as<std::string>());
	}

	return 0;
}
