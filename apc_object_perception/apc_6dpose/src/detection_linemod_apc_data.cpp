#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

#include <boost/any.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <v4r/common/pcl_opencv.h>

//#include <png++/png.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGBA;

pcl::LINEMOD linemod;


double score_threshold = 0.8;
void printElapsedTimeAndNumberOfPoints (double t, int w, int h=1)
{
  print_info ("[done, "); print_value ("%g", t); print_info (" ms : ");
  print_value ("%d", w*h); print_info (" points]\n");
}

bool
loadCloud (const std::string & filename, PointCloudXYZRGBA & cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);

  printElapsedTimeAndNumberOfPoints (tt.toc (), cloud.width, cloud.height);

  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

std::vector<pcl::LINEMODDetection>
matchTemplates (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input, const pcl::LINEMOD & linemod)
{
  pcl::ColorGradientModality<pcl::PointXYZRGB> color_grad_mod;
  color_grad_mod.setInputCloud (input);
  color_grad_mod.processInputData ();

  pcl::SurfaceNormalModality<pcl::PointXYZRGB> surface_norm_mod;
  surface_norm_mod.setInputCloud (input);
  surface_norm_mod.processInputData ();

  std::vector<pcl::QuantizableModality*> modalities (2);
  modalities[0] = &color_grad_mod;
  modalities[1] = &surface_norm_mod;

  std::vector<pcl::LINEMODDetection> detections;
  linemod.matchTemplates (modalities, detections);

  return (detections);
}

void compute (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & input) {

  // Match the templates to the provided image
  std::vector<pcl::LINEMODDetection> detections = matchTemplates (input, linemod);

  // Output the position and score of the best match for each template
  for (size_t i = 0; i < detections.size (); ++i) {
    const LINEMODDetection & d = detections[i];
    printf ("%lu: %d %d %d %f\n", i, d.x, d.y, d.template_id, d.score);
  }

  cv::Mat image = v4r::ConvertPCLCloud2Image(*input);
  // Draw a green box around the object
  for (size_t i = 0; i < detections.size (); ++i)
  {
    const LINEMODDetection & d = detections[i];
    if (d.score < score_threshold)
      continue;

    const pcl::SparseQuantizedMultiModTemplate & tmplt = linemod.getTemplate (d.template_id);
    std::cout << "d.template_id " << d.template_id << std::endl;
    int w = tmplt.region.width;
    int h = tmplt.region.height;
    cv::rectangle(image,  cv::Point(d.x, d.y),
          cv::Point(d.x + w , d.y + h),
          cv::Scalar(0,255,0));;
  }
  cv::imshow("detection", image);
  cv::waitKey(0);

  /*// Visualization code for testing purposes (requires libpng++)


  image.write("output.png");
  */
}

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main (int argc, char** argv) {
    std::string pathStr = ".";
    typedef pcl::PointXYZRGB PointT;
    std::string mask = "";
    int valid_model = 0;
    std::string template_filename = "";
    std::string class_name = "";



    po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("template,t", po::value(&template_filename)->required() , "Template filename generated from saveTemapltes")
        ("class,c", po::value(&class_name) , "If present will only look into one class")
        ("path,p",po::value(&pathStr)->required(),"Path to operate on")

            ;

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
     std::cout << "Loading model: " << template_filename << std::endl;
    linemod.loadTemplates(template_filename.c_str());
    for (int i = 0; i < filesPath.size(); i++){

        std::string maskFilename = "mask_"+class_name+".png";
        std::cout << "Recognizing file " << filesPath[i].string() << std::endl;
        if(!class_name.empty()){
            if(!fs::is_directory(filesPath[i]) || !fs::exists(filesPath[i] / maskFilename)){
                std::cout << "Class " << mask << " is not present " << std::endl;
                continue;
            }
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::io::loadPCDFile((filesPath[i] / "frame.pcd").string(), *input);

        // Load the specified templates and match them to the provided input cloud
        compute (input);

    }

}

