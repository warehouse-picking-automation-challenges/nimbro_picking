#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


#include <apc_6dpose/apc_6dpose.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>




pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr;
pcl::search::KdTree<pcl::PointXYZ> search;
pcl::PointCloud<pcl::PointXYZ>::Ptr xyzcloud;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
std::ofstream output_file;
namespace fs = boost::filesystem;
namespace po = boost::program_options;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void);
void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void);
void addAnnotationShape(pcl::PointXYZ pos,  pcl::PointXYZ direction, std::string id);
int num_neighbors;
std::vector<pcl::PointXYZ> grasp_points;
std::vector<pcl::PointXYZ> grasp_directions;
std::vector<std::string> id_queue;
double arrow_scale = 0.05;
double sphere_radius = 0.005;
std::string filename;



void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie)
{
    std::cout << "inside event" << std::endl;
    int idx = event.getPointIndex ();
    if (idx == -1)
        return;

    basic_cloud_ptr = *reinterpret_cast<pcl::PointCloud<pcl::PointXYZRGB>::Ptr*> (cookie);
    xyzcloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*basic_cloud_ptr, *xyzcloud);
    search.setInputCloud (xyzcloud);

    // Return the correct index in the cloud instead of the index on the screen
    std::vector<int> indices (1);
    std::vector<float> distances (1);

    // Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
    pcl::PointXYZ picked_pt;
    event.getPoint (picked_pt.x, picked_pt.y, picked_pt.z);
    search.nearestKSearch (picked_pt, 1, indices, distances);


    std::vector<int> pointIdxNKNSearch(num_neighbors);
    std::vector<float> pointNKNSquaredDistance(num_neighbors);
    if(!search.nearestKSearch (xyzcloud->points[indices[0]], num_neighbors, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
    return;
    }

    std::cout << "Calculating normals" << std::endl;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    float nx,ny,nz,c;
    ne.computePointNormal(*xyzcloud,pointIdxNKNSearch,nx,ny,nz,c);
    std::cout << "Computed normals" << std::endl;


    pcl::PointXYZRGB normal_end;
    pcl::PointXYZ normal_at_point(nx,ny,nz);


    normal_end.x = xyzcloud->points[indices[0]].x + nx*arrow_scale;
    normal_end.y = xyzcloud->points[indices[0]].y + ny*arrow_scale;
    normal_end.z = xyzcloud->points[indices[0]].z + nz*arrow_scale;

    std::cout << "Calculated things " << std::endl;
    std::stringstream ss;
    ss << xyzcloud->points[indices[0]] << normal_at_point;

    //ne.compute (*cloud_normals);



    PCL_INFO ("Point index picked: %d (real: %d) - [%f, %f, %f], and normal[%f,%f,%f]\n", idx, indices[0], picked_pt.x, picked_pt.y, picked_pt.z,nx,ny,nz);

    std::cout << "Check if viewer correct " << std::endl;
    std::cout << "Viewer alive " << viewer->wasStopped() << std::endl;

    if (viewer){
        grasp_points.push_back(xyzcloud->points[indices[0]]);
        grasp_directions.push_back(normal_at_point);
        id_queue.push_back(ss.str());
        addAnnotationShape(xyzcloud->points[indices[0]], normal_at_point, ss.str());

        pcl::PointXYZ pos;
        event.getPoint (pos.x, pos.y, pos.z);
        viewer->addText3D<pcl::PointXYZ> (ss.str (), pos, 0.0005, 1.0, 1.0, 1.0, ss.str ());
    }

}


void addAnnotationShape(pcl::PointXYZ pos,  pcl::PointXYZ direction, std::string id){
    if (viewer){

        pcl::PointXYZ normal_end;
        normal_end.x = pos.x + direction.x*arrow_scale;
        normal_end.y = pos.y + direction.y*arrow_scale;
        normal_end.z = pos.z + direction.z*arrow_scale;
        std::cout << "Viewer existx" << std::endl;
        viewer->addSphere (pos, sphere_radius, 255,0,0, "sphere_orig" + id);
        viewer->addSphere (normal_end, sphere_radius, 0, 0, 255, "sphere_end" + id);
        pcl::ModelCoefficients coeffs;
        coeffs.values.push_back (pos.x);
        coeffs.values.push_back (pos.y);
        coeffs.values.push_back (pos.z);

        //direction
        coeffs.values.push_back (direction.x*arrow_scale);
        coeffs.values.push_back (direction.y*arrow_scale);
        coeffs.values.push_back (direction.z*arrow_scale);

        coeffs.values.push_back (sphere_radius);
        viewer->addCylinder(coeffs, "cylinder" + id);
    }
}

void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" model.pcd outputfile.txt num_neighbors\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "To select a point to add: shift+left mouse click\n"
            << "To remove a point from the list, select it again with shift+left mouse click\n"
            << "Press r to remove last added point \n"
            << "Press m to flip normal \n"
            << "Press w to write to file \n"
            << "Press u to print this message \n"
            << "\n\n";
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
      viewer->addCoordinateSystem (0.02);
      viewer->initCameraParameters ();

      return (viewer);
}



unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getKeySym () == "r" && event.keyDown ()){
        std::cout << "r was pressed => removing previous point " << grasp_points.back() << " with normal " << grasp_directions.back() << std::endl;
        std::string id = id_queue.back();

        grasp_points.pop_back();
        grasp_directions.pop_back();
        id_queue.pop_back();

        viewer->removeShape("sphere_orig" + id );
        viewer->removeShape("sphere_end" + id );
        viewer->removeShape("cylinder" + id );
    }else if (event.getKeySym () == "m" && event.keyDown ()){
        std::cout << "m was pressed => flipping previous point " << grasp_points.back() << " with normal " << grasp_directions.back() << std::endl;
        std::string id = id_queue.back();

        pcl::PointXYZ flipped_normal = grasp_directions.back();
        pcl::PointXYZ point_orig = grasp_points.back();
        grasp_directions.pop_back();
        flipped_normal.x = -flipped_normal.x;
        flipped_normal.y = -flipped_normal.y;
        flipped_normal.z = -flipped_normal.z;

        grasp_directions.push_back(flipped_normal);

        viewer->removeShape("sphere_end" + id );
        viewer->removeShape("cylinder" + id );

        pcl::PointXYZ normal_end;
        normal_end.x = point_orig.x + flipped_normal.x*arrow_scale;
        normal_end.y = point_orig.y + flipped_normal.y*arrow_scale;
        normal_end.z = point_orig.z + flipped_normal.z*arrow_scale;

        viewer->addSphere (normal_end, sphere_radius, 0, 0, 255, "sphere_end" + id);
        pcl::ModelCoefficients coeffs;
        coeffs.values.push_back (point_orig.x);
        coeffs.values.push_back (point_orig.y);
        coeffs.values.push_back (point_orig.z);

        //direction
        coeffs.values.push_back (flipped_normal.x*arrow_scale);
        coeffs.values.push_back (flipped_normal.y*arrow_scale);
        coeffs.values.push_back (flipped_normal.z*arrow_scale);

        coeffs.values.push_back (sphere_radius);
        viewer->addCylinder(coeffs, "cylinder" + id);

    } else if (event.getKeySym () == "w" && event.keyDown ()){
        std::cout << " writing file " << filename << std::endl;
        if(!output_file.is_open()){
            output_file.open (filename, ios::app ); //trying to open the file again in append mode
            if(!output_file.is_open()){
                std::cout <<"Cannot save file, probably it was already closed or never opened" << std::endl;
                return;
            }
        }
        for(int i =0; i < grasp_points.size(); i++){
            output_file  <<  grasp_points[i].x << "," <<  grasp_points[i].y << "," <<  grasp_points[i].z << ","
                         << grasp_directions[i].x << "," << grasp_directions[i].y << "," << grasp_directions[i].z  << " \n";
        }
        output_file.close();
    } else if (event.getKeySym () == "u" && event.keyDown ()){
        printUsage("grasp_annotation");
    }

}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);

  }

}


void readGraspingPositions(const std::string filename, std::vector<pcl::PointXYZ> &grasp_points, std::vector<pcl::PointXYZ> &grasp_directions){
    std::ifstream grasp_file(filename);
    std::string line;
    while (std::getline(grasp_file, line)) {
        if (boost::starts_with(line, "#"))
            continue;
        std::stringstream          lineStream(line);
        std::string                cell;
        std::vector<float> values;
        while(std::getline(lineStream,cell, ',')){
            values.push_back(atof(cell.c_str()));
        }
        if(values.size()!=6){
            std::cout << "Error, line " << line << " is not correctly defined"  << std::endl;
        }else{
            pcl::PointXYZ position(values[0],values[1],values[2]);
            pcl::PointXYZ direction(values[3],values[4],values[5]);
            grasp_points.push_back(position);
            grasp_directions.push_back(direction);
        }
    }
}

int main (int argc, char** argv){

    std::string intialization_file = "";
    std::string input_file = "";
    std::string output_filename = "";
    num_neighbors = 100;
    po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("input,i", po::value<std::string>(&input_file)->required(), "Input pcd to annotate")
        ("output,o", po::value<std::string>(&output_filename)->required(), "Output file")
        ("initialization", po::value<std::string>(&intialization_file), "Initialization file")
        ("num_neighbors,n",po::value<int>(&num_neighbors),"Number of neighbors to calculate normals")

   ;

    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    po::store(parsed, vm);
    if (vm.count("help")) { std::cout << desc << std::endl; }
    try { po::notify(vm); }
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }


    readGraspingPositions(intialization_file, grasp_points, grasp_directions);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    fs::path pcdPath(input_file);
    filename = output_filename;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original;
    original.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    basic_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    if(pcl::io::loadPCDFile((pcdPath).string(), *original) < 0){
        fprintf(stderr, "Could not load PCD file\n");
        return 1;
    }


    if(!filename.empty()){
        std::cout << "Creating file " << filename << std::endl;
        output_file.open (filename);
        output_file << "#Grasping pose for model ." << pcdPath.string() <<"\n";
        output_file << "#Point\tVector" << pcdPath.string() <<"\n";
    }

    //Subsample cloud so that is easy to selet each point
    pcl::PCLPointCloud2::Ptr pcl_orig (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr pcl_filtered (new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(*original, *pcl_orig);
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (pcl_orig);
    sor.setLeafSize (0.008f, 0.008f, 0.008f);
    sor.filter (*pcl_filtered );
    pcl::fromPCLPointCloud2(*pcl_filtered, *basic_cloud_ptr);


    viewer = simpleVis(basic_cloud_ptr);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);
    viewer->registerPointPickingCallback (&pp_callback, static_cast<void*> (&basic_cloud_ptr));


    for(int i = 0 ; i < grasp_points.size(); i++){
        std::stringstream ss;
        ss << grasp_points[i] << grasp_directions[i];
        id_queue.push_back(ss.str());
        addAnnotationShape(grasp_points[i], grasp_directions[i], ss.str());
    }

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
