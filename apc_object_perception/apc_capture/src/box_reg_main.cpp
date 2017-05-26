#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <std_msgs/Float64.h>


// Since PCL causes some compiler warnings, disable sign-compare and deprecation
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"

#undef UNDEF_DEPRECATED
#ifdef __DEPRECATED
#define UNDEF_DEPRECATED
#undef __DEPRECATED
#endif

#  include <pcl/visualization/pcl_visualizer.h>
#  include <pcl/registration/gicp.h>
#  include <pcl/filters/conditional_removal.h>
#  include <pcl/conversions.h>
#  include <pcl/filters/passthrough.h>
#  include <pcl/filters/voxel_grid.h>

#ifdef UNDEF_DEPRECATED
#define __DEPRECATED
#undef UNDEF_DEPRECATED
#endif

#pragma GCC diagnostic pop


template<typename PointT>
void loadPointcloudXYZRGB(std::string pcdfile,pcl::PointCloud<PointT>& cloud )
{
	if (pcl::io::loadPCDFile<PointT>(pcdfile,cloud) == -1)
	{
		ROS_ERROR("Falied to read the PCL file");
	} else
	{
		ROS_INFO_STREAM("The PCD file loaded");
	}

}


void convertpclXYZRGB2XYZ(pcl::PointCloud<pcl::PointXYZRGB>& cloudXYZRGB , pcl::PointCloud<pcl::PointXYZ>& cloudXYZ)
{
	for (size_t i = 0; i < cloudXYZRGB.points.size(); i++)
	{
		cloudXYZ.points[i].x = cloudXYZRGB.points[i].x;
		cloudXYZ.points[i].y = cloudXYZRGB.points[i].y;
		cloudXYZ.points[i].z = cloudXYZRGB.points[i].z;
	}
}



/*Steps : 1. Determine which box the current input belongs to.
 *  this is just interpreting the result of ICP
 * 2. with the box determined, we can precompute the coordinates of the vertices of the box
 * 3. with these vertices we can get the min distance to the shelf
 */
void GetDistace2ShelfFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
	pcl::PointXYZ min,max;
	pcl::getMinMax3D(*cloud,min,max);
	std::cout<<" Min "<<min<<" max "<<max<<std::endl;

	// the box correspondences are already determined
	// For each box we hv mix X,Y,z and , max X,Y,Z
	// (i.e) min X is 0.006 and Max X is 0.421

	int i,j; // i assume mapping from box label to box indices exists and i will use those values later
	// i corresponds to vertical index and J corresponds to horizontal index
	// the top most cell has i =0 and left most cell has j=0
	i=1;  // column (2)
	j=1; // row (3)this is to be replaced soon // probably as funtion argument

	// Start with X axis
	// at least min X and min Y is same for all boxes
	float min_X = 0.006f;
	float max_X = 0.421f;

	// next do for y
	float base_Y = -0.425f;
	float min_Y,max_Y;
	float seperator_thickness = 0.006f;

	max_Y = base_Y - seperator_thickness ; // start with subtracting seperator_thickness. This is done to nullify the addition of
	// seperator_thickness done at first step of the loop
	for (int t=0;t<=i;t++)
	{
		min_Y = max_Y + seperator_thickness;
		if ( t%2 == 0 ) // for outer boxes
		{
			max_Y = min_Y + 0.27f;
		}
		else
		{
			max_Y = min_Y + 0.298f;
		}

	}
	std::cout<<"Min_Y "<<min_Y<<" "<<" max_Y "<<max_Y<<std::endl;
	// the horizontal seperator thickness is 0.004
	seperator_thickness = 0.004f;

	float base_Z = 1.803f ;
	float min_Z,max_Z;

	min_Z = base_Z + seperator_thickness;
	// same like Y. But start with adding Y since we ll be adding subtracting  it in the loops
	for (int t=0;t<=j;t++)
	{
		max_Z = min_Z - seperator_thickness;
		if ( t%3 == 0 ) // first and last box are of same dimension and inner two are same
		{
			min_Z = max_Z - 0.263f;
		}
		else
		{
			min_Z = max_Z - 0.224f;
		}

	}
	std::cout<<std::setprecision(4)<<" max_z "<<max_Z<<" Min_Z "<<min_Z<<std::endl;
	std::vector<Eigen::Vector4f> dist2shelf;
	//Just for visualizing --start
	float min_minDist = std::numeric_limits<float>::infinity();
	float max_minDist = 0;
	// --end
	// Now start with feature computation



	for(size_t t = 0;t < cloud->points.size();t++)
	{

		float min_dist = std::min({ std::min( std::abs(cloud->points[t].x - min_X) , std::abs(max_X - cloud->points[t].x)  ),
									std::min( std::abs(cloud->points[t].y - min_Y) , std::abs(max_Y - cloud->points[t].y)  ),
									std::min( std::abs(cloud->points[t].z - min_Z) , std::abs(max_Z - cloud->points[t].z)  )});
		dist2shelf.push_back(Eigen::Vector4f (cloud->points[t].x,cloud->points[t].y,cloud->points[t].z,min_dist ));
		if (min_dist > max_minDist)
			max_minDist = min_dist;
		if (min_dist < min_minDist)
			min_minDist = min_dist;
	}
	ROS_INFO_STREAM("Features feature done");
	ROS_INFO_STREAM("MIn_minDist "<<min_minDist<<" max_minDist "<<max_minDist);



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr featureVisualize (new pcl::PointCloud<pcl::PointXYZRGB>);
	featureVisualize->width = cloud->width;
	featureVisualize->height = cloud->height;
	featureVisualize->points.resize(cloud->width * cloud->height);

	std::cout<<" dist2shelf "<<dist2shelf.size()<<std::endl;
	std::cout<<" "<<dist2shelf[1]<<std::endl;

	for(size_t t=0; t < dist2shelf.size(); t++)
	{

		featureVisualize->points[t].x = dist2shelf[t][0];
		featureVisualize->points[t].y = dist2shelf[t][1];
		featureVisualize->points[t].z = dist2shelf[t][2];
		float temp = 255 * ( dist2shelf [t][3] ) / ( max_minDist );
		featureVisualize->points[t].r = temp;
		featureVisualize->points[t].g = 255-temp;
		featureVisualize->points[t].b = 0;

		//  std::cout<<" "<<(int)featureVisualize->points[t].r<<" "<<(int)featureVisualize->points[t].g<<" "<<(int)featureVisualize->points[t].b<<" "<< (int)temp <<std::endl;

	}
	std::string rosPackagePath = ros::package::getPath("apc_capture");
	std::string imagePath = rosPackagePath + "/resource/pcd/";
	pcl::io::savePCDFile(imagePath+"output.pcd",*featureVisualize);

	pcl::visualization::PCLVisualizer featureviewer ("Feature visualization");
	featureviewer.setBackgroundColor (128, 128, 255);
	featureviewer.addPointCloud<pcl::PointXYZRGB> (featureVisualize, "Feature");
	featureviewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Feature");
	featureviewer.addCoordinateSystem (1.0);

	while (!featureviewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		featureviewer.spinOnce ();
	}


	std::cout<<" Returning from fuction"<<std::endl;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Box_registration");
	ros::NodeHandle nh("~");
	ROS_INFO_STREAM("Node is Up");
	std::string rosPackagePath = ros::package::getPath("apc_capture");
	std::string imagePath = rosPackagePath + "/resource/pcd/";

	// load First file
	std::string imageName = "0004.pcd";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudfromSensor (new pcl::PointCloud<pcl::PointXYZRGB>);
	loadPointcloudXYZRGB(imagePath+imageName,*cloudfromSensor);

	ROS_INFO_STREAM("Cloud Height "<<cloudfromSensor->height<<" Cloud Width "<<cloudfromSensor->width);
	ROS_INFO_STREAM("Cloud points size "<<cloudfromSensor->points.size());


	//load Second file
	imageName = "meshCloudCoorCorrected.pcd";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromMesh (new pcl::PointCloud<pcl::PointXYZ>);
	loadPointcloudXYZRGB(imagePath+imageName,*cloudfromMesh);

	ROS_INFO_STREAM("Cloud Height "<<cloudfromMesh->height<<" Cloud Width "<<cloudfromMesh->width);
	ROS_INFO_STREAM("Cloud points size "<<cloudfromMesh->points.size());


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromSensorXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	cloudfromSensorXYZ->width = cloudfromSensor->width;
	cloudfromSensorXYZ->height = cloudfromSensor->height;
	cloudfromSensorXYZ->points.resize(cloudfromSensor->width * cloudfromSensor->height);
	convertpclXYZRGB2XYZ(*cloudfromSensor,*cloudfromSensorXYZ);
	ROS_INFO_STREAM("Conversion done");

	Eigen::Affine3f transform;
	transform = Eigen::Translation3f(-0.6, 0, 1.4) *
				Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitZ()) *
				Eigen::AngleAxisf(250.0 * M_PI / 180.0, Eigen::Vector3f::UnitX());





	// Apply the transformation to cloudfromSensorXYZ

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloudfromSensorXYZ,*cloudTemp,transform);
	cloudfromSensorXYZ = cloudTemp;

	//remove outliers


	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 1.18)));

	pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
	condrem.setInputCloud (cloudfromSensorXYZ);
	condrem.setKeepOrganized(true);
	// apply filter
	condrem.filter (*cloudTemp);
	cloudfromSensorXYZ = cloudTemp;




	// set some initial guess -- later this will be replaced by the values from the TF
	Eigen::Matrix4f initialguess;
	initialguess = Eigen::Matrix4f::Identity();

	// Pre-processing to remove NaN values
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromSensorfiltered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromMeshfiltered (new pcl::PointCloud<pcl::PointXYZ>);


	pcl::PassThrough<pcl::PointXYZ> pass; // can do this without parameters
	pass.setInputCloud( cloudfromSensorXYZ );
	//pass.setFilterFieldName ("z"); // only depth values will have Nan (hv to check on this)
	pass.filter( *cloudfromSensorfiltered );


	pass.setInputCloud( cloudfromMesh );
	pass.filter( *cloudfromMeshfiltered );


	// Downsampling the could points
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromSensorSampled (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromMeshSampled (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> voxgrid;
	voxgrid.setInputCloud (cloudfromSensorfiltered);
	voxgrid.setLeafSize (0.01f, 0.01f, 0.01f);
	voxgrid.filter (*cloudfromSensorSampled);

	voxgrid.setInputCloud (cloudfromMeshfiltered);
	voxgrid.setLeafSize (0.01f, 0.01f, 0.01f);
	voxgrid.filter (*cloudfromMeshSampled);


	// ICP

	ros::Time begin = ros::Time::now();

	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloudfromSensorSampled);
	icp.setInputTarget(cloudfromMeshSampled);
	pcl::PointCloud<pcl::PointXYZ> Final;


	icp.align(Final,initialguess);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	ros::Time end = ros::Time::now();
	std::cout<<"Time ICP took :  "<<end-begin;

	// Apply transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtransformed (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloudfromSensorSampled,*cloudtransformed,icp.getFinalTransformation() );


	std::cout << " Starting Visualization" << std::endl;
	// try visualizing
	pcl::visualization::PCLVisualizer viewer ("ICP visualization");


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloudfromSensorSampled, 255, 255, 255);
	viewer.addPointCloud (cloudfromSensorSampled, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloudtransformed, 230, 20, 20); // Red
	viewer.addPointCloud (cloudtransformed, transformed_cloud_color_handler, "transformed_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mesh_cloud_color_handler (cloudfromMeshSampled, 0, 0, 255);
	viewer.addPointCloud (cloudfromMeshSampled, mesh_cloud_color_handler, "Mesh_cloud");


	viewer.addCoordinateSystem (1.0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Mesh_cloud");
	//viewer.setPosition(800, 400); // Setting visualiser window position


	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}


	// Apply transformation to the original cloud  ( cloudfromSensorXYZ) and not the downsampled


	pcl::transformPointCloud(*cloudfromSensorXYZ,*cloudtransformed,icp.getFinalTransformation() );
	GetDistace2ShelfFeature(cloudtransformed);


	/*
	pcl::visualization::PCLVisualizer viewer1 ("BOX visualization");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler1 (cloudfromSensorSampled, 255, 255, 255);
	viewer1.addPointCloud (cloudtransformed, transformed_cloud_color_handler1, "original_cloud");
	viewer1.addCoordinateSystem (1.0, 0);
	viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer1.spinOnce ();
	}


	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	std::string nodelet_name = ros::this_node::getName();
	nodelet.load(nodelet_name, "apc_capture/test_nodelet", remap, nargv);
	*/

	return 0;
}





