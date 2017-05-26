#include <apc_capture/apc_features.h>
#include <apc_objects/apc_objects.h>
#include <apc_capture/box_registration.h>
#include <eigen_conversions/eigen_msg.h>
#include <limits.h>

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
#  include <pcl/filters/fast_bilateral.h>
#  include <pcl/keypoints/uniform_sampling.h>
#  include <pcl/features/integral_image_normal.h>

#ifdef UNDEF_DEPRECATED
#define __DEPRECATED
#undef UNDEF_DEPRECATED
#endif

#pragma GCC diagnostic pop

static constexpr bool VISUALIZE = false;

namespace apc_object_perception
{

ApcFeatures::ApcFeatures()
{
	m_boxIndices(0) = -1;
	m_boxIndices(1) = -1; // to check later if initialized
}

ApcFeatures::~ApcFeatures()
{
}

void ApcFeatures::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	m_cloudfromSensorXYZRGB = cloud;
}

void ApcFeatures::setBoxCoordinates(apc_capture::BoxCoordinates& msg)
{
	m_boxmsg = msg;
}

apc_capture::BoxCoordinates ApcFeatures::getBoxCoordinates()
{
	return m_boxmsg;
}

void ApcFeatures::setCameraModel(const sensor_msgs::CameraInfoConstPtr& msg)
{
	m_cam_model.fromCameraInfo(msg);
}

void ApcFeatures::setBoxIndices(const Eigen::Vector2i& box)
{
	m_boxIndices = box;
	computeBoxCoordinates();
	m_isBoxIndicesSet = true;
}

void ApcFeatures::computeBoxCoordinates()
{
	float min_X = -0.421f;
	float max_X = -0.006f;
	float base_Y = 0.425f;
	float min_Y(0), max_Y(0);
	float seperator_thickness = 0.006;

	int i,j; //i --> column j --> row
	i = m_boxIndices[1];
	j = m_boxIndices[0];

	min_Y = base_Y + seperator_thickness ; // start with subtracting seperator_thickness. This is done to nullify the addition of
	// seperator_thickness done at first step of the loop
	for (int t=0;t<=i;t++)
	{
		max_Y = min_Y - seperator_thickness;
		if ( t%2 == 0 ) // for outer boxes
		{
			min_Y = max_Y - 0.27f;
		}
		else
		{
			min_Y = max_Y - 0.298f;
		}
	}
	// the horizontal seperator thickness is 0.004
	seperator_thickness = 0.004f;

	float base_Z = 1.803f ;
	float min_Z(0), max_Z(0);

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

	m_boxmsg.boxCoordinates[0] = min_X;
	m_boxmsg.boxCoordinates[1] = max_X;
	m_boxmsg.boxCoordinates[2] = min_Y;
	m_boxmsg.boxCoordinates[3] = max_Y;
	m_boxmsg.boxCoordinates[4] = min_Z;
	m_boxmsg.boxCoordinates[5] = max_Z;
}

void ApcFeatures::setMeshCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudfromMesh)
{
	m_meshCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

	{
		pcl::PointCloud<int> indices;

		pcl::UniformSampling<pcl::PointXYZ> voxgrid;
		voxgrid.setInputCloud(cloudfromMesh);
		voxgrid.setRadiusSearch(m_leafSize);
		voxgrid.compute(indices);

		m_meshCloud->reserve(indices.size());
		for(int idx : indices)
			m_meshCloud->push_back((*cloudfromMesh)[idx]);
	}

	// Pre-calculate Kdtree
	m_icp.setInputTarget(m_meshCloud);
}

Eigen::Affine3f ApcFeatures::doRegistration(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudfromSensorXYZ,
											const Eigen::Affine3f& transformSensorInWorldFrame,
											const cv::Mat_<uint8_t>& shelfMask,
											Eigen::Affine3f* correctedShelf
											)
{
	using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

	if (!m_isBoxIndicesSet)
		throw std::runtime_error("Box Indices not set");

	PointCloud::Ptr sensorCloudInWorld(new PointCloud);
	pcl::transformPointCloud(*cloudfromSensorXYZ, *sensorCloudInWorld, transformSensorInWorldFrame);

	Eigen::Matrix4f initialguess =  Eigen::Matrix4f::Identity();

	pcl::PointXYZ min,max;
	pcl::getMinMax3D(*sensorCloudInWorld, min, max);
	std::cout << " PointCloud Min: " << min << ", max: " << max << std::endl;

	std::cout << " Box Coordinates X: " << m_boxmsg.boxCoordinates[0] << " " << m_boxmsg.boxCoordinates[1] << std::endl;
	std::cout << " Box Coordinates Y: " << m_boxmsg.boxCoordinates[2] << " " << m_boxmsg.boxCoordinates[3] << std::endl;
	std::cout << " Box Coordinates Z: " << m_boxmsg.boxCoordinates[4] << " " << m_boxmsg.boxCoordinates[5] << std::endl;



	// Filter out object points / external points with some heuristics
	if(true)
	{
// 		pcl::PointXYZ invalid;
// 		invalid.x = invalid.y = invalid.z = NAN;
		PointCloud::Ptr tmp(new PointCloud/*(sensorCloudInWorld->width, sensorCloudInWorld->height, invalid)*/);

		// filter out pixels that belongs to objects based on shelf mask
		for(size_t i = 0; i < sensorCloudInWorld->size(); ++i)
		{
			auto& point = (*sensorCloudInWorld)[i];
			if(shelfMask(i / sensorCloudInWorld->width, i % sensorCloudInWorld->width) == 0)
			{
				// the point belongs to an object
				point.x = point.y = point.z = NAN;
			}
		}

		float error_limit = 0.08f  ;// *** check this***
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("x", pcl::ComparisonOps::GT, apc_objects::distRobot2Shelf + m_boxmsg.boxCoordinates[0] - 2.0f*error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("x", pcl::ComparisonOps::LT, apc_objects::distRobot2Shelf + m_boxmsg.boxCoordinates[1] + 2.0f*error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("y", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[2] - error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("y", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[3] + error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("z", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[4] - error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("z", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[5] + error_limit));

		pcl::ConditionalRemoval<pcl::PointXYZ> removal (range_cond);
		removal.setInputCloud (sensorCloudInWorld);
		removal.setKeepOrganized(true);

		removal.filter (*tmp);

		/*// compute Min/Max along x axis

		float max = std::numeric_limits<float>::min();
		float min = std::numeric_limits<float>::max();
		for(size_t i = 0; i < tmp->size(); ++i)
		{
			auto &point = (*tmp)[i];
			if(point.x > max)
				max = point.x;
			if(point.x < min)
				min = point.x;
		}
		std::cout<<" min / max" << min<<" "<<max<<std::endl;*/

		// compute robust min values along x axis
		// compute min values in upper and lower half separately and consider only the points closer to the upper and lower box plane
#if 0
		std::vector<float> cloudX_upperHalf,cloudX_lowerHalf;
		for(size_t i = 0; i < tmp->size(); ++i)
		{
			auto& point = (*tmp)[i];
			if(!pcl::isFinite(point))
				continue;
			if(point.z <  (m_boxmsg.boxCoordinates[4] + m_boxmsg.boxCoordinates[5]) / 2)
			{
				//if (std::abs(point.z - m_boxmsg.boxCoordinates[4])  < 0.06)
					cloudX_lowerHalf.push_back(point.x);
			}
			else
			{
				//if (std::abs(point.z - m_boxmsg.boxCoordinates[5])  < 0.06)
					cloudX_upperHalf.push_back(point.x);
			}

		}

		std::sort(cloudX_lowerHalf.begin(),cloudX_lowerHalf.end());
		std::sort(cloudX_upperHalf.begin(),cloudX_upperHalf.end());

		float min_upperHalfX,min_lowerHalfX;

		min_upperHalfX = cloudX_upperHalf[0.01 * cloudX_upperHalf.size()];
		min_lowerHalfX = cloudX_lowerHalf[0.01 * cloudX_lowerHalf.size()];

		// Filter out any points definitely in the interior
		for(size_t i = 0; i < tmp->size(); ++i)
		{
			auto& point = (*tmp)[i];
			if(!pcl::isFinite(point))
				continue;

			Eigen::Vector3f inBox(
				point.x - apc_objects::distRobot2Shelf - m_boxmsg.boxCoordinates[0],
				point.y - m_boxmsg.boxCoordinates[2],
				point.z - m_boxmsg.boxCoordinates[4]
			);

			if(inBox.x() > 0.04 && (point.y - m_boxmsg.boxCoordinates[2] > 0.04 && m_boxmsg.boxCoordinates[3] - point.y > 0.04))
			{
				point.x = point.y = point.z = NAN;
				continue;
			}

			float dist2Shelf = std::min({

				apc_objects::distRobot2Shelf + m_boxmsg.boxCoordinates[1] - point.x,

				point.y - m_boxmsg.boxCoordinates[2],
				m_boxmsg.boxCoordinates[3] - point.y,

				point.z - m_boxmsg.boxCoordinates[4],
				m_boxmsg.boxCoordinates[5] - point.z
			});

			if(dist2Shelf > 0.04 && m_boxmsg.boxCoordinates[5] - point.z > 0.08)
			{
				point.x = point.y = point.z = NAN;
			}




			// Filter out points futher away from the front face
			// only the points along the narrow strip closer to mid height of the box is kept if it is away from the front face

			if (point.z < (m_boxmsg.boxCoordinates[4] + m_boxmsg.boxCoordinates[5]) / 2)
			{
				//lowerHalf
				if(point.x > min_lowerHalfX + 0.02)
				{
					if (std::abs(point.z -  ((m_boxmsg.boxCoordinates[4] +  m_boxmsg.boxCoordinates[5]) / 2) ) > 0.04)
						point.x = point.y = point.z = NAN;
				}

				// filter the points that lies before the min_lowerHalfX along x axis
				if(point.x < min_lowerHalfX)
					point.x = point.y = point.z = NAN;
			}
			else
			{
				// upperHalf
				if(point.x > min_upperHalfX + 0.02)
				{
					if (std::abs(point.z -  ((m_boxmsg.boxCoordinates[4] +  m_boxmsg.boxCoordinates[5]) / 2) ) > 0.04)
						point.x = point.y = point.z = NAN;
				}

				// filter the points that lies before the min_upperHalfX along x axis
				if(point.x < min_upperHalfX)
					point.x = point.y = point.z = NAN;
			}
		}
#endif

		sensorCloudInWorld = tmp;

/*		// cut out the points on the lower half of the box
		range_cond.reset(new pcl::ConditionAnd<pcl::PointXYZ> ());
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("z", pcl::ComparisonOps::GT, (m_boxmsg.boxCoordinates[4] + m_boxmsg.boxCoordinates[5]) / 2));
		pcl::ConditionalRemoval<pcl::PointXYZ> removalLowerHalf (range_cond);
		removalLowerHalf.setInputCloud(sensorCloudInWorld);
		removalLowerHalf.setKeepOrganized(true);
		removalLowerHalf.filter (*tmp);
		sensorCloudInWorld = tmp;*/

		// Filter out any points futher away from the front face


	}

	// Interpolate the top face if it was not visible
	if(false)
	{
		PointCloud interpolationPoints;

		for(size_t x = 0; x < sensorCloudInWorld->width; ++x)
		{
			pcl::PointXYZ last;
			last.z = NAN;

			for(size_t y = sensorCloudInWorld->height-2; y > sensorCloudInWorld->height - 600; --y)
			{
				auto cur = (*sensorCloudInWorld)(x, y);

				if(!pcl::isFinite(last))
					last = cur;

				if(pcl::isFinite(cur))
				{
					if(cur.x - last.x > 0.04 && ( cur.y >  m_boxmsg.boxCoordinates[2] +  0.02 && cur.y < m_boxmsg.boxCoordinates[3] - 0.02 ) )
					{
						int steps = (cur.x - last.x) / 0.005;
						for(int i = 1; i < steps; ++i)
						{
							auto interpolated = last;
							interpolated.getVector3fMap() += i * (cur.getVector3fMap() - last.getVector3fMap()) / steps;

							interpolationPoints.push_back(interpolated);
						}
					}

					last = cur;
				}
			}
		}

		// NOTE: This destroys the organization...
		(*sensorCloudInWorld) += interpolationPoints;
	}

	printf(" preprocessing...\n");
	{
		auto filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		pcl::FastBilateralFilter<pcl::PointXYZ> filter;
		filter.setInputCloud(sensorCloudInWorld);
		filter.filter(*filtered);

		sensorCloudInWorld = filtered;
	}

	// Pre-processing to remove NaN values
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromSensorfiltered (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> pass; // can do this without parameters
	pass.setInputCloud( sensorCloudInWorld );
	pass.filter( *cloudfromSensorfiltered );

	// Downsampling the could points
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromSensorSampled (new pcl::PointCloud<pcl::PointXYZ>);
	{
		pcl::PointCloud<int> indices;

		pcl::UniformSampling<pcl::PointXYZ> voxgrid;
		voxgrid.setInputCloud(cloudfromSensorfiltered);
		voxgrid.setRadiusSearch(m_leafSize);
		voxgrid.compute(indices);

		cloudfromSensorSampled->header = cloudfromSensorXYZ->header;
		cloudfromSensorSampled->header.frame_id = "world";
		cloudfromSensorSampled->reserve(indices.size());
		for(int idx : indices)
			cloudfromSensorSampled->push_back((*cloudfromSensorfiltered)[idx]);
	}
	printf(" preprocessing done.\n");
	m_filteredCloud = cloudfromSensorSampled;

// 	m_icp.setCorrespondenceRandomness(10);
// 	m_icp.setMaximumOptimizerIterations(80);

	Eigen::Affine3f preprocessTransform = Eigen::Affine3f::Identity();
// 	preprocessTransform = Eigen::Translation3f(
// 		-(m_boxmsg.boxCoordinates[0] + m_boxmsg.boxCoordinates[1])/2 - apc_objects::distRobot2Shelf,
// 		-(m_boxmsg.boxCoordinates[2] + m_boxmsg.boxCoordinates[3])/2,
// 		-(m_boxmsg.boxCoordinates[4] + m_boxmsg.boxCoordinates[5])/2
// 	);

	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredShelf(new pcl::PointCloud<pcl::PointXYZ>);
	if(true)
	{
		float error_limit = 0.08f  ;// *** check this***
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("x", pcl::ComparisonOps::GT, apc_objects::distRobot2Shelf + m_boxmsg.boxCoordinates[0] - 2.0f*error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("x", pcl::ComparisonOps::LT, apc_objects::distRobot2Shelf + m_boxmsg.boxCoordinates[1] + 2.0f*error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("y", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[2] - error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("y", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[3] + error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("z", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[4] - error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("z", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[5] + error_limit));

		pcl::ConditionalRemoval<pcl::PointXYZ> removal (range_cond);
		removal.setInputCloud(m_meshCloud);
		removal.setKeepOrganized(false);

		removal.filter (*filteredShelf);

		std::cout << "Shelf filter: " << m_meshCloud->size() << " => " << filteredShelf->size() << "\n";
// 		m_icp.setInputTarget(filteredShelf);
	}
	else
		filteredShelf = m_meshCloud;

	filteredShelf->header.frame_id = "world";
	filteredShelf->header.stamp = cloudfromSensorSampled->header.stamp;

	m_filteredMeshCloud = filteredShelf;

	pcl::PointCloud<pcl::PointXYZ>::Ptr translatedShelf(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*filteredShelf, *translatedShelf, preprocessTransform);

	pcl::PointCloud<pcl::PointXYZ>::Ptr translatedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloudfromSensorSampled, *translatedCloud, preprocessTransform);

	pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
	pcl::PointCloud<pcl::PointXYZ> Final;
	m_icp.setInputSource(translatedCloud);
	m_icp.setInputTarget(translatedShelf);
// 	m_icp.setMaxCorrespondenceDistance(0.05);
// 	m_icp.setTransformationEpsilon(0.0000001);
// 	m_icp.setRotationEpsilon(0.00001);
	m_icp.align(Final,initialguess);
	pcl::console::setVerbosityLevel(pcl::console::L_INFO);

	Eigen::Affine3f icpResult(m_icp.getFinalTransformation());
	icpResult = preprocessTransform.inverse() * icpResult * preprocessTransform;

	std::cout<<m_icp.getFinalTransformation() << std::endl;
	std::cout<<"ICP has converged ? "<< std::boolalpha<<m_icp.hasConverged()<<std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtransformed (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloudfromSensorSampled,*cloudtransformed, icpResult);

	m_transformedCloud = cloudtransformed;

	// try visualizing
	if(VISUALIZE)
	{
		// Apply transformation
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
	/*	pcl::transformPointCloud(*cloudfromSensorXYZ, *tmp, transformSensorInWorldFrame);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtransformed (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*tmp,*cloudtransformed,m_icp.getFinalTransformation() );*/

		// For debugging



		float error_limit = 0.08f  ;// *** check this***
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("x", pcl::ComparisonOps::GT, apc_objects::distRobot2Shelf + m_boxmsg.boxCoordinates[0] - 2.0f*error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("x", pcl::ComparisonOps::LT, apc_objects::distRobot2Shelf + m_boxmsg.boxCoordinates[1] + 2.0f*error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("y", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[2] - error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("y", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[3] + error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("z", pcl::ComparisonOps::GT, m_boxmsg.boxCoordinates[4] - error_limit));
		range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ>>("z", pcl::ComparisonOps::LT, m_boxmsg.boxCoordinates[5] + error_limit));

		pcl::ConditionalRemoval<pcl::PointXYZ> removal (range_cond);
		removal.setInputCloud(cloudtransformed);
		removal.setKeepOrganized(true);

		removal.filter (*tmp);

		cloudtransformed = tmp;

		pcl::visualization::PCLVisualizer viewer ("ICP visualization");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloudfromSensorSampled, 255, 0, 0);
		viewer.addPointCloud (cloudfromSensorSampled, source_cloud_color_handler, "original_cloud");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloudtransformed, 255, 255, 255);
		viewer.addPointCloud (cloudtransformed, transformed_cloud_color_handler, "transformed_cloud");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mesh_cloud_color_handler (filteredShelf, 0, 0, 255);
		viewer.addPointCloud (filteredShelf, mesh_cloud_color_handler, "Mesh_cloud");

		viewer.addCoordinateSystem (1.0, transformSensorInWorldFrame);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Mesh_cloud");
		//viewer.setPosition(800, 400); // Setting visualiser window position

		viewer.setCameraPosition(
			0.0, 0.0, 2.0,
			apc_objects::distRobot2Shelf + (m_boxmsg.boxCoordinates[0] + m_boxmsg.boxCoordinates[1])/2,
			(m_boxmsg.boxCoordinates[2] + m_boxmsg.boxCoordinates[3])/2,
			(m_boxmsg.boxCoordinates[4] + m_boxmsg.boxCoordinates[5])/2,
			0.0, 0.0, 1.0
		);

		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce ();
		}

		viewer.close();
		viewer.spinOnce();
	}

	initialguess = Eigen::Affine3f(Eigen::Translation3f(apc_objects::distRobot2Shelf,0,0)).matrix() ;// get the initial guess again

	// ICP transformation: corrected frame in world frame

	// Shelf frame in corrected frame
	Eigen::Affine3f shelfFrame (icpResult.inverse() * initialguess);

	// Sensor frame in shelf frame (corrected)
	Eigen::Affine3f camPoints2Shelf = shelfFrame.inverse() * transformSensorInWorldFrame;

	if(correctedShelf)
		*correctedShelf = shelfFrame;

	return camPoints2Shelf;
}
void ApcFeatures::createBinMask(Eigen::Affine3f shelfToCameraFrame,cv::Mat& binMaskImage)
{
  // check if Pointcloud,cameramodel,box are set
	if((m_boxIndices[0] == -1 || m_boxIndices[1] == -1) ||!m_cam_model.initialized())
	{
		ROS_ERROR_STREAM("Camera Model or Box Indices not set");
		throw std::runtime_error("cam model or box indices not set");
	}

	binMaskImage = cv::Mat::zeros(
		m_cam_model.fullResolution().height,
		m_cam_model.fullResolution().width,
		CV_8UC1
	);


	typedef std::vector<cv::Point> point_vector;
	typedef std::vector<point_vector> contour_vector;
	contour_vector contours(1);

	cv::Point3d boxPoint;

	
	//bottom left
	boxPoint.x = m_boxmsg.boxCoordinates[0];
	boxPoint.y = m_boxmsg.boxCoordinates[3];
	boxPoint.z = m_boxmsg.boxCoordinates[4];

	Eigen::Vector4f vec(boxPoint.x,boxPoint.y,boxPoint.z,1);
	vec = ( shelfToCameraFrame.matrix() * vec);
	boxPoint.x = vec.x();
	boxPoint.y = vec.y();
	boxPoint.z = vec.z();

	cv::Point2d returnPoint = m_cam_model.project3dToPixel(boxPoint);
	contours[0].push_back(returnPoint);

//bottom right
	boxPoint.x = m_boxmsg.boxCoordinates[0];
	boxPoint.y = m_boxmsg.boxCoordinates[2];
	boxPoint.z = m_boxmsg.boxCoordinates[4];

	vec = Eigen::Vector4f(boxPoint.x,boxPoint.y,boxPoint.z,1);
	vec = ( shelfToCameraFrame.matrix() * vec);
	boxPoint.x = vec.x();
	boxPoint.y = vec.y();
	boxPoint.z = vec.z();

	returnPoint = m_cam_model.project3dToPixel(boxPoint);
	contours[0].push_back(returnPoint);

	//top right
	boxPoint.x = m_boxmsg.boxCoordinates[0];
	boxPoint.y = m_boxmsg.boxCoordinates[2];
	boxPoint.z = m_boxmsg.boxCoordinates[5];

	vec  = Eigen::Vector4f(boxPoint.x,boxPoint.y,boxPoint.z,1);
	vec = ( shelfToCameraFrame.matrix() * vec);
	boxPoint.x = vec.x();
	boxPoint.y = vec.y();
	boxPoint.z = vec.z();

	returnPoint = m_cam_model.project3dToPixel(boxPoint);
	contours[0].push_back(returnPoint);
	
	//top left
	boxPoint.x = m_boxmsg.boxCoordinates[0];
	boxPoint.y = m_boxmsg.boxCoordinates[3];
	boxPoint.z = m_boxmsg.boxCoordinates[5];

	vec = Eigen::Vector4f(boxPoint.x,boxPoint.y,boxPoint.z,1);
	vec = ( shelfToCameraFrame.matrix() * vec);
	boxPoint.x = vec.x();
	boxPoint.y = vec.y();
	boxPoint.z = vec.z();

	returnPoint = m_cam_model.project3dToPixel(boxPoint);
	contours[0].push_back(returnPoint);

	cv::drawContours(binMaskImage,contours,-1,255,-1);

	//cv::imshow("bin",binMaskImage);
	//cv::waitKey(0);
}

void ApcFeatures::setRGBImage(cv::Mat& in)
{
	this->rgbImage = in.clone();

}

void ApcFeatures::computeDist2ShelfFeature(cv::Mat_<float>& result)
{
	// check if Pointcloud,cameramodel,box are set
	if( !m_cloudfromSensorXYZRGB || (m_boxIndices[0] == -1 || m_boxIndices[1] == -1) ||!m_cam_model.initialized() )
	{
		ROS_ERROR_STREAM("Camera Model or Box Indices or Point Cloud not set");
	}

	result      =    cv::Mat_<float> (m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width);
	for(size_t t = 0;t < m_cloudfromSensorXYZRGB->points.size();t++)
	{
		auto& p = m_cloudfromSensorXYZRGB->points[t];

		float min_dist = std::min({
			// Front face is open => measure only distance to back face
			m_boxmsg.boxCoordinates[1] - p.x,

			p.y - m_boxmsg.boxCoordinates[2],
			m_boxmsg.boxCoordinates[3] - p.y,

			p.z - m_boxmsg.boxCoordinates[4],
			m_boxmsg.boxCoordinates[5] - p.z
		});

		// Cap to zero (or almost zero to allow for errors)
		min_dist = std::max(min_dist, -0.01f);

		result(t/m_cloudfromSensorXYZRGB->width, t%m_cloudfromSensorXYZRGB->width) = min_dist ;
	}
}


void ApcFeatures::computeHeightFeatures(Eigen::Affine3f& tfTransIn,cv::Mat_<float>& height3Dresult,
										cv::Mat_<float>& MissingHeightappromation, cv::Mat& isValidHeight ,
										cv::Mat_<float>& depth)
{
	if( !m_cloudfromSensorXYZRGB || (m_boxIndices[0] == -1 || m_boxIndices[1] == -1) ||!m_cam_model.initialized() )
	{
		ROS_ERROR_STREAM("Camera Model or Box Indices or Point Cloud not set");
		return ;
	}
	float plane_d = -0.421;
	height3Dresult             = cv::Mat_<float> (m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width);
	MissingHeightappromation   = cv::Mat_<float> (m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width);
	depth					   = cv::Mat_<float> (m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width);
	isValidHeight              = cv::Mat(m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width, CV_8UC1);
	Eigen::Vector3f camera = tfTransIn.translation();
	std::cout<<" camera translation "<<camera.transpose()<<std::endl;
	for(uint32_t y = 0; y < m_cloudfromSensorXYZRGB->height ; y++)
	{
		for(uint32_t x=0;x < m_cloudfromSensorXYZRGB->width ; x++)
		{
			depth(y,x) = m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].z;
			if (std::isfinite(m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].z) )
			{
				isValidHeight.at<uchar>(y,x) = 1;
				MissingHeightappromation(y,x) = NAN;

				float heightInBox = m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].z - m_boxmsg.boxCoordinates[4];

				height3Dresult(y,x) = std::max(heightInBox, -0.02f);
			}
			else
			{
				isValidHeight.at<uchar>(y,x)= 0;
				height3Dresult(y,x) = NAN;

				//tf::vectorTFToEigen(tfTransIn.translation(),camera);
				cv::Point3d cv_ray = m_cam_model.projectPixelTo3dRay(cv::Point2d(x,y));

				Eigen::Vector3f ray(cv_ray.x, cv_ray.y, cv_ray.z);

				// Transform ray into shelf frame
				ray = tfTransIn.rotation() * ray;

				// camera.x + lambda * ray.x = d
				// compute lambda. This lambda will be hieght from the ground plane
				// subtract the height of box plane to get the height of the pixel
				float lambda = ( plane_d - camera.x() ) / ray.x();
				MissingHeightappromation(y, x) = camera.z() + lambda * ray.z() - m_boxmsg.boxCoordinates[4];
			}
		}
	}
}

void ApcFeatures::computeEdge(cv::Mat& EdgeOut)
{
	EdgeOut  = cv::Mat(m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width, CV_8UC1);
	cv::Mat rgbImage( m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width, CV_8UC3);

	//generate rgb image from pointcloud
	for(uint32_t y = 0; y < m_cloudfromSensorXYZRGB->height ; y++)
	{
		for(uint32_t x=0;x < m_cloudfromSensorXYZRGB->width ; x++)
		{
			rgbImage.at<cv::Vec3b>(y,x) = cv::Vec3b(m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].r,
													m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].g,
													m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].b);
		}
	}
	cv::Mat gray_image,canny_edges;
	cvtColor( rgbImage, gray_image, CV_RGB2GRAY );
	cv::Canny(gray_image,canny_edges /* EdgeOut */,80,240,3); //***  check these values ***

	int dilation_size = 5; //*** from RBO ***
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
													cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
													cv::Point( dilation_size, dilation_size ) );
	dilate( canny_edges, EdgeOut, element );

}


void ApcFeatures::computeMissingHeight(Eigen::Affine3f& tfTransIn,cv::Mat_<float> &result)
{
	float plane_d = -0.421;
	result =  cv::Mat_<float> (m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width);
	for(uint32_t y = 0; y < m_cloudfromSensorXYZRGB->height ; y++)
	{
		for(uint32_t x=0;x < m_cloudfromSensorXYZRGB->width ; x++)
		{
			if (! std::isfinite(m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].z) )
			{
				Eigen::Vector3f camera = tfTransIn.translation();
				std::cout<<" camera translation "<<camera<<std::endl;
				//tf::vectorTFToEigen(tfTransIn.translation(),camera);
				cv::Point3d cv_ray = m_cam_model.projectPixelTo3dRay(cv::Point2d(x,y));
				// camera.x + lambda * ray.x = d
				// compute lambda. This lambda will be hieght from the ground plane
				// subtract the height of box plane to get the height of the pixel
				float lambda = ( plane_d - camera(0) ) / cv_ray.x;
				result(y, x) = camera.z() + lambda * cv_ray.z - m_boxmsg.boxCoordinates[4];
			}
			else
			{
				result(y,x) = NAN;
			}
		}
	}
}

void ApcFeatures::computeHeight3D(cv::Mat_<float> &result)
{
	result = cv::Mat_<float> (m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width);
	for(uint32_t y = 0; y < m_cloudfromSensorXYZRGB->height ; y++)
	{
		for(uint32_t x=0;x < m_cloudfromSensorXYZRGB->width ; x++)
		{
			if (std::isfinite(m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].z) )
			{
				result(y,x) = m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].z;
			}
			else
			{
				result(y,x) = NAN;
			}
		}
	}
}

void ApcFeatures::computeIsValid3D(cv::Mat &result)
{
	result = cv::Mat(m_cloudfromSensorXYZRGB->height,m_cloudfromSensorXYZRGB->width, CV_8UC1);
	for(uint32_t y = 0; y < m_cloudfromSensorXYZRGB->height ; y++)
	{
		for(uint32_t x=0;x < m_cloudfromSensorXYZRGB->width ; x++)
		{
			if (std::isfinite(m_cloudfromSensorXYZRGB->points[ y * m_cloudfromSensorXYZRGB->width + x ].z) )
			{
				result.at<uchar>(y,x) = 1;
			}
			else
			{
				result.at<uchar>(y,x) = 0;
			}
		}
	}
}

cv::Mat ApcFeatures::computeHHA(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData)
{
	cv::Mat_<cv::Vec3b> result(m_cloudfromSensorXYZRGB->height, m_cloudfromSensorXYZRGB->width);

	pcl::PointCloud<pcl::Normal> normalCloud;
	{
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

		ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor(0.8f);
		ne.setNormalSmoothingSize(30.0f);

		ne.setInputCloud(rawData);
		ne.compute(normalCloud);
	}

	for(unsigned int y = 0; y < m_cloudfromSensorXYZRGB->height; ++y)
	{
		for(unsigned int x = 0; x < m_cloudfromSensorXYZRGB->width; ++x)
		{
			auto& rawPoint = (*rawData)(x,y);
			auto& normalPoint = normalCloud(x,y);

			if(!pcl::isFinite(rawPoint) || !pcl::isFinite(normalPoint))
				result(y,x) = cv::Vec3b(0, 0, 0);
			else
			{
				Eigen::Vector3f point = cameraPose * rawPoint.getVector3fMap();

				const float MIN_DEPTH = 0.2;
				const float MAX_DEPTH = 0.7;
				const float MAX_HEIGHT = 0.4;

				float disparity = (255.0f / (MAX_DEPTH/MIN_DEPTH)) * (MAX_DEPTH / std::max(rawPoint.z, MIN_DEPTH));
				float height = (255.0f / MAX_HEIGHT) * (point.z() - m_boxmsg.boxCoordinates[4]);

				Eigen::Vector3f normalInCamFrame = normalPoint.getNormalVector3fMap();
				Eigen::Vector3f normalInShelfFrame = cameraPose.rotation() * normalInCamFrame;

				float angle = std::acos(normalInShelfFrame.z()) - M_PI/2.0f;

				float angleToGravity = 128.0f + (90.0f / (M_PI/2.0f)) * angle;

				result(y,x) = cv::Vec3b(
					cv::saturate_cast<uint8_t>(angleToGravity),
					cv::saturate_cast<uint8_t>(height),
					cv::saturate_cast<uint8_t>(disparity)
				);
			}
		}
	}

	return result;
}

cv::Mat ApcFeatures::computeHHAV(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData, const cv::Mat_<uint8_t>& mask)
{
	cv::Mat_<cv::Vec3b> result(m_cloudfromSensorXYZRGB->height, m_cloudfromSensorXYZRGB->width);

	pcl::PointCloud<pcl::Normal> normalCloud;
	{
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

		ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor(0.8f);
		ne.setNormalSmoothingSize(30.0f);

		ne.setInputCloud(rawData);
		ne.compute(normalCloud);
	}

	for(unsigned int y = 0; y < m_cloudfromSensorXYZRGB->height; ++y)
	{
		for(unsigned int x = 0; x < m_cloudfromSensorXYZRGB->width; ++x)
		{
			auto& rawPoint = (*rawData)(x,y);
			auto& normalPoint = normalCloud(x,y);

			if(!pcl::isFinite(rawPoint) || !pcl::isFinite(normalPoint) || !mask(y,x))
				result(y,x) = cv::Vec3b(0, 0, 0);
			else
			{
				Eigen::Vector3f point = cameraPose * rawPoint.getVector3fMap();

				const float MIN_DEPTH = 0.2;
				const float MAX_DEPTH = 0.7;
				const float MAX_HEIGHT = 0.4;

				float disparity = (255.0f / (MAX_DEPTH/MIN_DEPTH)) * (MAX_DEPTH / std::max(rawPoint.z, MIN_DEPTH));
				float height = (255.0f / MAX_HEIGHT) * (point.z() - m_boxmsg.boxCoordinates[4]);

				Eigen::Vector3f normalInCamFrame = normalPoint.getNormalVector3fMap();
				Eigen::Vector3f normalInShelfFrame = cameraPose.rotation() * normalInCamFrame;

				float angle = std::acos(normalInShelfFrame.z()) - M_PI/2.0f;

				float angleToGravity = 128.0f + (90.0f / (M_PI/2.0f)) * angle;

				result(y,x) = cv::Vec3b(
					cv::saturate_cast<uint8_t>(angleToGravity),
					cv::saturate_cast<uint8_t>(height),
					cv::saturate_cast<uint8_t>(disparity)
				);
			}
		}
	}

	return result;
}


cv::Mat ApcFeatures::computeHVA(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData, const cv::Mat_<uint8_t>& mask)
{
	cv::Mat_<cv::Vec3b> result(m_cloudfromSensorXYZRGB->height, m_cloudfromSensorXYZRGB->width);

	pcl::PointCloud<pcl::Normal> normalCloud;
	{
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

		ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor(0.8f);
		ne.setNormalSmoothingSize(30.0f);

		ne.setInputCloud(rawData);
		ne.compute(normalCloud);
	}

	for(unsigned int y = 0; y < m_cloudfromSensorXYZRGB->height; ++y)
	{
		for(unsigned int x = 0; x < m_cloudfromSensorXYZRGB->width; ++x)
		{
			auto& rawPoint = (*rawData)(x,y);
			auto& normalPoint = normalCloud(x,y);

			if(!pcl::isFinite(rawPoint) || !pcl::isFinite(normalPoint))
				result(y,x) = cv::Vec3b(0, 0, 0);
			else
			{
				const float MIN_DEPTH = 0.2;
				const float MAX_DEPTH = 0.7;

				float disparity = (255.0f / (MAX_DEPTH/MIN_DEPTH)) * (MAX_DEPTH / std::max(rawPoint.z, MIN_DEPTH));

				Eigen::Vector3f normalInCamFrame = normalPoint.getNormalVector3fMap();
				Eigen::Vector3f normalInShelfFrame = cameraPose.rotation() * normalInCamFrame;

				float angle = std::acos(normalInShelfFrame.z()) - M_PI/2.0f;

				float angleToGravity = 128.0f + (90.0f / (M_PI/2.0f)) * angle;

				result(y,x) = cv::Vec3b(
					cv::saturate_cast<uint8_t>(angleToGravity),
					mask(y,x),
					cv::saturate_cast<uint8_t>(disparity)
				);
			}
		}
	}

	return result;
}

cv::Mat ApcFeatures::computeHHV(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData, const cv::Mat_<uint8_t>& mask)
{
	cv::Mat_<cv::Vec3b> result(m_cloudfromSensorXYZRGB->height, m_cloudfromSensorXYZRGB->width);

	for(unsigned int y = 0; y < m_cloudfromSensorXYZRGB->height; ++y)
	{
		for(unsigned int x = 0; x < m_cloudfromSensorXYZRGB->width; ++x)
		{
			auto& rawPoint = (*rawData)(x,y);

			if(!pcl::isFinite(rawPoint))
				result(y,x) = cv::Vec3b(0, 0, 0);
			else
			{
				Eigen::Vector3f point = cameraPose * rawPoint.getVector3fMap();

				const float MIN_DEPTH = 0.2;
				const float MAX_DEPTH = 0.7;
				const float MAX_HEIGHT = 0.4;

				float disparity = (255.0f / (MAX_DEPTH/MIN_DEPTH)) * (MAX_DEPTH / std::max(rawPoint.z, MIN_DEPTH));
				float height = (255.0f / MAX_HEIGHT) * (point.z() - m_boxmsg.boxCoordinates[4]);

				result(y,x) = cv::Vec3b(
					mask(y,x),
					cv::saturate_cast<uint8_t>(height),
					cv::saturate_cast<uint8_t>(disparity)
				);
			}
		}
	}

	return result;
}

} // end namespace apc_object_perception
