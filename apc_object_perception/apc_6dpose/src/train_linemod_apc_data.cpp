#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>


#include <pcl/point_types.h>


#include <iostream>
#include <sstream>

#include <boost/any.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>
#include <math.h>
#include <algorithm>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


void
trainTemplate (const PointCloud::ConstPtr & input,  cv::Mat_<uint8_t> &foreground_mask,
               pcl::LINEMOD & linemod)
{

    std::cout << "Processing color modality" << std::endl;
  pcl::ColorGradientModality<pcl::PointXYZRGB> color_grad_mod;
  color_grad_mod.setInputCloud (input);
  color_grad_mod.processInputData ();

  std::cout << "processing normal modality" << std::endl;

  pcl::SurfaceNormalModality<pcl::PointXYZRGB> surface_norm_mod;
  surface_norm_mod.setInputCloud (input);
  surface_norm_mod.processInputData ();

  std::cout << "processing quantizable" << std::endl;

  std::vector<pcl::QuantizableModality*> modalities (2);
  modalities[0] = &color_grad_mod;
  modalities[1] = &surface_norm_mod;

  size_t min_x (input->width), min_y (input->height), max_x (0), max_y (0);
  pcl::MaskMap mask_map (input->width, input->height);
  int nCols = foreground_mask.cols;
  int nRows = foreground_mask.rows;
  uint8_t* p;
  std::cout << "nRows " << nRows << " nCols" << nCols << std::endl;
  int counter = 0;
  for(size_t i = 0; i < nRows; ++i){
      p = foreground_mask.ptr<uint8_t>(i);
      for (size_t j = 0; j < nCols; ++j){
          mask_map (j,i) = (((int) p[j])==255);
          if(((int) p[j])==255){
            min_x = std::min (min_x, j);
            max_x = std::max (max_x, j);
            min_y = std::min (min_y, i);
            max_y = std::max (max_y, i);
          }
      }
  }

  std::cout << "finished with masks process" << std::endl;

  std::vector<pcl::MaskMap*> masks (2);
  std::cout << "variable declaration " << std::endl;
  masks[0] = &mask_map;
  masks[1] = &mask_map;

  std::cout << "processing region" << std   ::endl;
  pcl::RegionXY region;
  region.x = static_cast<int> (min_x);
  region.y = static_cast<int> (min_y);
  region.width = static_cast<int> (max_x - min_x + 1);
  region.height = static_cast<int> (max_y - min_y + 1);

  printf ("%d %d %d %d\n", region.x, region.y, region.width, region.height);


  linemod.createAndAddTemplate (modalities, masks, region);
}


    int main(int argc, char** argv)
    {

        std::string pathStr = ".";
        typedef pcl::PointXYZRGB PointT;
        std::string mask = "";
        int valid_model = 0;


        po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("mask,m", po::value<std::string>(&mask), "Mask input cloud with given class")
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
        std::string maskFilename = "mask_"+mask+".png";

        pcl::LINEMOD linemod;
        int counter = 0;

        for (int i = 0; i < filesPath.size(); i++){
            std::cout << "Recognizing file " << filesPath[i].string() << std::endl;
            PointCloud::Ptr input(new PointCloud);
            PointCloud::Ptr whole_cloud(new PointCloud);

            pcl::io::loadPCDFile((filesPath[i] / "frame.pcd").string(), *input);
            pcl::copyPointCloud(*input, *whole_cloud);
            if(!mask.empty()){
                if(!fs::is_directory(filesPath[i]) || !fs::exists(filesPath[i] / maskFilename)){
                    std::cout << "Class " << mask << " is not present " << std::endl;
                    continue;
                }

                cv::Mat_<uint8_t> mask_box = cv::imread((filesPath[i] / maskFilename).string(), CV_LOAD_IMAGE_GRAYSCALE);
                std::cout << "Read " << (filesPath[i] / maskFilename).string() << std::endl;
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
                        if(((int) p[j])==255){
                            indices.indices.push_back(i*nCols + j);
                            counter++;
                        }
                    }
                }


                pcl::ExtractIndices<PointT> extract_indices;
                extract_indices.setIndices (boost::make_shared<const pcl::PointIndices> (indices));
                extract_indices.setInputCloud (input);
                extract_indices.setKeepOrganized (true);
                extract_indices.filterDirectly(input);
                std::cout << "Output has " << input->points.size () << " points." << std::endl;
                std::cout << "indices size " << indices.indices.size()<< " vs counter" << counter << std::endl;
                pcl::io::savePCDFile((filesPath[i] / ("frame_" + mask +"_ " + std::to_string(valid_model) +"_template.pcd")).string(), *input);
                trainTemplate (whole_cloud, mask_box, linemod);
                std::cout << "finished" << std::endl;


                // Save the LINEMOD template
                std::ofstream file_stream;
                std::string template_sqmmt_filename = (filesPath[i] / ("frame_" + mask +"_template.sqmmt")).string();
                file_stream.open (template_sqmmt_filename.c_str (), std::ofstream::out | std::ofstream::binary);
                linemod.getTemplate (valid_model).serialize (file_stream);
                file_stream.close ();
                valid_model++;

            }


        }

        std::cout << "writing all tamples to " << pathStr <<"/" <<mask <<"_template.lmt"<< std::endl;
        linemod.saveTemplates((pathStr + "/" + mask + "_template.lmt").c_str());
        return 0;
    }
