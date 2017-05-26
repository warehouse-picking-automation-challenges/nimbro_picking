#include <v4r/common/miscellaneous.h>  // to extract Pose intrinsically stored in pcd file

#include <v4r/io/filesystem.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>

#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <iostream>
#include <sstream>

#include <boost/any.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>

#include <opencv2/opencv.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

    int main(int argc, char** argv)
    {

        std::string pathStr = ".";
        std::string modelPath = ".";
        std::string className = ".";


        unsigned int finished = 0;

        typedef pcl::PointXYZRGB PointT;
        typedef v4r::Model<PointT> ModelT;
        typedef boost::shared_ptr<ModelT> ModelTPtr;

        typedef pcl::PointXYZRGB PointT;
        std::string test_dir;
        bool visualize = true;
        double chop_z = std::numeric_limits<double>::max();
        std::string mask = "";

        google::InitGoogleLogging(argv[0]);

        po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("visualize,v", po::bool_switch(&visualize), "visualize recognition results")
            ("mask,m", po::value<std::string>(&mask), "Mask input cloud with given class")
            ("chop_z,z", po::value<double>(&chop_z)->default_value(chop_z, boost::str(boost::format("%.2e") % chop_z) ), "points with z-component higher than chop_z_ will be ignored (low chop_z reduces computation time and false positives (noise increase with z)")
            ("path,p",po::value(&pathStr)->required(),"Path to operate on")
       ;

        std::cout << "Flag 1" << std::endl;
        po::variables_map vm;
        po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
        po::store(parsed, vm);
        if (vm.count("help")) { std::cout << desc << std::endl; }
        try { po::notify(vm); }
        catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }

        // Gather list of pcd
        std::vector<fs::path> filesPath;
        {
            auto it = fs::recursive_directory_iterator(pathStr);
            auto end = fs::recursive_directory_iterator();

            for(; it != end; ++it)
            {
                fs::path path = *it;
                if(fs::is_directory(path) && fs::exists(path / "frame.pcd"))
                    filesPath.push_back(fs::canonical(path));
            }
        }
        v4r::MultiRecognitionPipeline<PointT> r(argc, argv);
        std::string maskFilename = "mask_"+mask+".png";

        for (int i = 0; i < filesPath.size(); i++){
            std::cout << "Recognizing file " << filesPath[i].string() << std::endl;
            LOG(INFO) << "Recognizing file " << filesPath[i];
            PointCloud::Ptr input(new PointCloud);
            PointCloud::Ptr output(new PointCloud);

            pcl::io::loadPCDFile((filesPath[i] / "frame.pcd").string(), *input);
            if(!mask.empty()){
                if(!fs::is_directory(filesPath[i]) || !fs::exists(filesPath[i] / maskFilename)){
                    std::cout << "Class " << mask << " is not present " << std::endl;
                    continue;
                }

                cv::Mat_<uint8_t> mask_box = cv::imread((filesPath[i] / maskFilename).string(), CV_LOAD_IMAGE_GRAYSCALE);
                //cv::imshow("Mask", mask_box);
                //cv::waitKey(0);
                std::cout << "Output has " << input->points.size () << " points." << std::endl;
                pcl::PointIndices indices;
                int nCols = mask_box.cols;
                int nRows = mask_box.rows;
                uint8_t* p;
                std::cout << "nRows " << nRows << " nCols" << nCols << std::endl;
                int counter = 0;
                for(int i = 0; i < nRows; ++i){
                    p = mask_box.ptr<uint8_t>(i);
                    for (int j = 0; j < nCols; ++j){
                        //std::cout << ((int) p[j]) << std::endl;
                        if(((int) p[j])==255){
                            indices.indices.push_back(i*nCols + j);
                            counter++;
                            //std::cout << "correct point" << std::endl;
                        }
                    }
                }


                pcl::ExtractIndices<PointT> extract_indices;
                extract_indices.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
                extract_indices.setInputCloud (input);
                extract_indices.setKeepOrganized (true);
                extract_indices.filterDirectly(input);
                //extract_indices.setUserFilterValue();
                //extract_indices.filter (*input);
                pcl::io::savePCDFileASCII ("/home/aura/apc/test_pcd.pcd", *input);
                std::cout << "Output has " << input->points.size () << " points." << std::endl;
                std::cout << "indices size " << indices.indices.size()<< " vs counter" << counter << std::endl;
            }


            if( chop_z > 0)
            {
                pcl::PassThrough<PointT> pass;
                pass.setFilterLimits ( 0.f, chop_z );
                pass.setFilterFieldName ("z");
                pass.setInputCloud (input);
                pass.setKeepOrganized (true);
                pass.filter (*input);
            }

            r.setInputCloud (input);
            r.recognize();

            std::vector<ModelTPtr> verified_models = r.getVerifiedModels();
            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_verified;
            transforms_verified = r.getVerifiedTransforms();

            if (visualize){
                std::cout << "about to visualize" << std::endl;
                r.visualize();
            }

            for(size_t m_id=0; m_id<verified_models.size(); m_id++)
                LOG(INFO) << "********************" << verified_models[m_id]->id_ << std::endl;

        }

        return 0;
    }
