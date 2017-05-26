#include <apc_capture/apc_tote_features.h>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <apc_shelf_model/dimensions.h>
#include "utils.cpp"
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
#  include <pcl/features/integral_image_normal.h>
#ifdef UNDEF_DEPRECATED
#define __DEPRECATED
#undef UNDEF_DEPRECATED
#endif

#pragma GCC diagnostic pop

static constexpr bool VISUALIZE = false;

namespace  apc_object_perception
{

ApcToteFeatures::ApcToteFeatures()
{
}

ApcToteFeatures::~ApcToteFeatures()
{
}
void ApcToteFeatures::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    m_cloudFromSensorXYZRGB   = cloud;

}

void ApcToteFeatures::setMeshCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudfromMesh)
{
    m_meshCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxgrid;

    voxgrid.setInputCloud (cloudfromMesh);
    voxgrid.setLeafSize (0.01f, 0.01f, 0.01f);
    voxgrid.filter(*m_meshCloud);
    // Pre-calculate Kdtree
    m_icp.setInputTarget(m_meshCloud);
}

Eigen::Affine3f ApcToteFeatures::doRegistration(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudfromSensorXYZ,
                                            const Eigen::Affine3f& transformSensorInWorldFrame,
                                            Eigen::Affine3f* correctedBox)
{
    pointCloudXYZ::Ptr sensorCloudInWorld(new pointCloudXYZ);
    pcl::transformPointCloud(*cloudfromSensorXYZ, *sensorCloudInWorld, transformSensorInWorldFrame);

    Eigen::Matrix4f initialguess =  Eigen::Matrix4f::Identity();

    pcl::PointXYZ min,max;
    pcl::getMinMax3D(*sensorCloudInWorld, min, max);
    std::cout << " PointCloud Min: " << min << ", max: " << max << std::endl;


    // the values need to be alterd for box
    // Filter out object points / external points with some heuristics
    if(true)
    {
        pointCloudXYZ::Ptr tmp(new pointCloudXYZ);

        //float error_limit = 0.08f  ;// *** check this***
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, -0.32))); // -0.3 (workaround for inaccurate tote STL )
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT,  0.32)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, -0.23)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT,  0.23)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT,  0.15)));
        //range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT,  0.25)));


        pcl::ConditionalRemoval<pcl::PointXYZ> removal (range_cond);
        removal.setInputCloud (sensorCloudInWorld);
        removal.setKeepOrganized(true);

        removal.filter (*tmp);


        /*// Filter out any points definitely in the interior of the box
        // and remove all the points that lie closer to center in z axis
        for(size_t i = 0; i < tmp->size(); ++i)
        {
            auto &point = (*tmp)[i];
            if (!pcl::isFinite(point))
                continue;

            if ((std::abs(point.x) < 0.2 && std::abs(point.y) < 0.1) || (point.z >  0.025 && point.z < 0.15)) {
                point.x = point.y = point.z = NAN;
            }


        }*/

        sensorCloudInWorld = tmp;
    }


    // color based filtering

    cv::Mat yuv;
    cv::cvtColor(this->rgbImage,yuv,CV_BGR2YUV);
    cv::Mat yuvChannels[3];
    cv::split(yuv,yuvChannels);
    for (int y = 0;y<rgbImage.rows;++y)
    {
        for (int x=0;x<rgbImage.cols;++x)
        {
            auto &point =(*sensorCloudInWorld)[y*1920 + x];
            if ( !( ( (yuvChannels[1].at<uchar>(y,x) > 128) && (yuvChannels[1].at<uchar>(y,x) < 162) ) && ( (yuvChannels[2].at<uchar>(y,x) > 96)  && (yuvChannels[2].at<uchar>(y,x) < 128) ) ))
                point.x = point.y = point.z = NAN;

        }
    }

    pcl::PassThrough<pcl::PointXYZ> pass; // can do this without parameters

    pointCloudXYZ::Ptr meshCloudCopy(new pointCloudXYZ);
    pcl::copyPointCloud(*m_meshCloud,*meshCloudCopy);
    /*// Filter out any points definitely in the interior of the mesh
    for(size_t i = 0; i < meshCloudCopy->size(); ++i)
    {
        auto &point = (*meshCloudCopy)[i];
        if (!pcl::isFinite(point))
            continue;

        if (std::abs(point.x) < 0.2 && std::abs(point.y) < 0.1) {
            point.x = point.y = point.z = NAN;
        }
    }*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(m_meshCloud);
    pass.filter(*meshCloudFiltered);


    printf(" preprocessing...\n");
    // Pre-processing to remove NaN values
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromSensorfiltered (new pcl::PointCloud<pcl::PointXYZ>);


    pass.setInputCloud( sensorCloudInWorld );
    pass.filter( *cloudfromSensorfiltered );

    // Downsampling the could points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfromSensorSampled (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxgrid;
    voxgrid.setInputCloud (cloudfromSensorfiltered);
    voxgrid.setLeafSize (0.01f, 0.01f, 0.01f);
    voxgrid.filter (*cloudfromSensorSampled);
    printf(" preprocessing done.\n");


    pcl::PointCloud<pcl::PointXYZ> Final;

    pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
    m_icp.setCorrespondenceRandomness(10);
    m_icp.setMaximumOptimizerIterations(80);
    m_icp.setInputSource(cloudfromSensorSampled);
    m_icp.setMaxCorrespondenceDistance(0.05);
    m_icp.setTransformationEpsilon(0.0000001);
    m_icp.setRotationEpsilon(0.00001);
    m_icp.align(Final,initialguess);

    std::cout<<m_icp.getFinalTransformation() << std::endl;
    std::cout<<"IPC has converged ? "<< std::boolalpha<<m_icp.hasConverged()<<std::endl;

    Eigen::Affine3f icpResult(m_icp.getFinalTransformation());

    if(VISUALIZE)
    {
        // Apply transformation
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
/*        pcl::transformPointCloud(*cloudfromSensorXYZ, *tmp, transformSensorInWorldFrame);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtransformed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*tmp,*cloudtransformed,m_icp.getFinalTransformation() );*/

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtransformed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloudfromSensorSampled,*cloudtransformed, icpResult);

        pcl::visualization::PCLVisualizer viewer ("ICP visualization");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloudfromSensorSampled, 255, 0, 0);
        viewer.addPointCloud (cloudfromSensorSampled, source_cloud_color_handler, "original_cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloudtransformed, 255, 255, 255);
        viewer.addPointCloud (cloudtransformed, transformed_cloud_color_handler, "transformed_cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> mesh_cloud_color_handler (meshCloudFiltered, 0, 0, 255);
        viewer.addPointCloud (meshCloudFiltered, mesh_cloud_color_handler, "Mesh_cloud");

        viewer.addCoordinateSystem (1.0, transformSensorInWorldFrame);
        viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "transformed_cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Mesh_cloud");

       viewer.setCameraPosition(
                0.0, 0.0, 2.0,
                0.0, 0.0, 0.0, 1
        );

        while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
            viewer.spinOnce ();
        }
    }

    initialguess = Eigen::Affine3f(Eigen::Translation3f(0,0,0)).matrix() ;// get the initial guess again

    // ICP transformation: corrected frame in world frame
    Eigen::Affine3f icpTrans(m_icp.getFinalTransformation());

    // box frame in corrected frame
    Eigen::Affine3f boxFrame (icpTrans.inverse() * initialguess);

    // Sensor frame in box frame (corrected)
    Eigen::Affine3f camPoints2Box = boxFrame.inverse() * transformSensorInWorldFrame;

    if(correctedBox)
        *correctedBox = boxFrame;

    return camPoints2Box;
}

void ApcToteFeatures::createToteMask(Eigen::Affine3f toteToCameraFrame,cv::Mat& toteMaskImage)
{
    if(!m_cam_model.initialized())
    {
        ROS_ERROR_STREAM("Camera Model not set");
        throw std::runtime_error("cam model not set");
    }
    toteMaskImage = cv::Mat::zeros(
            m_cam_model.fullResolution().height,
            m_cam_model.fullResolution().width,
            CV_8UC1
    );

    typedef std::vector<cv::Point> point_vector;
    typedef std::vector<point_vector> contour_vector;
    contour_vector contours(1);

    cv::Point3d totePoint;

    float tote_X = 0.30f, tote_Y = 0.18f, tote_Z = 0.2f;

    // top right (+ +)
    totePoint.x = tote_X;
    totePoint.y = tote_Y;
    totePoint.z = tote_Z;

    Eigen::Vector4f vec(totePoint.x,totePoint.y,totePoint.z,1);
    vec = ( toteToCameraFrame.matrix() * vec);
    totePoint.x = vec.x();
    totePoint.y = vec.y();
    totePoint.z = vec.z();

    cv::Point2d returnPoint = m_cam_model.project3dToPixel(totePoint);
    contours[0].push_back(returnPoint);


    // top left (- +)
    totePoint.x = -1 * tote_X;
    totePoint.y = tote_Y;
    totePoint.z = tote_Z;

    vec = Eigen::Vector4f(totePoint.x,totePoint.y,totePoint.z,1);
    vec = ( toteToCameraFrame.matrix() * vec);
    totePoint.x = vec.x();
    totePoint.y = vec.y();
    totePoint.z = vec.z();

    returnPoint = m_cam_model.project3dToPixel(totePoint);
    contours[0].push_back(returnPoint);

    // bottom left (- -)
    totePoint.x = -1 * tote_X;
    totePoint.y = -1 * tote_Y;
    totePoint.z = tote_Z;

    vec = Eigen::Vector4f(totePoint.x,totePoint.y,totePoint.z,1);
    vec = ( toteToCameraFrame.matrix() * vec);
    totePoint.x = vec.x();
    totePoint.y = vec.y();
    totePoint.z = vec.z();

    returnPoint = m_cam_model.project3dToPixel(totePoint);
    contours[0].push_back(returnPoint);


    // bottom right  (+ -)
    totePoint.x = tote_X;
    totePoint.y = -1 * tote_Y;
    totePoint.z = tote_Z;

    vec = Eigen::Vector4f(totePoint.x,totePoint.y,totePoint.z,1);
    vec = ( toteToCameraFrame.matrix() * vec);
    totePoint.x = vec.x();
    totePoint.y = vec.y();
    totePoint.z = vec.z();

    returnPoint = m_cam_model.project3dToPixel(totePoint);
    contours[0].push_back(returnPoint);

    cv::drawContours(toteMaskImage,contours,-1,255,-1);

}
void ApcToteFeatures::computeHeightFeatures(cv::Mat_<float> &height3Dresult, cv::Mat &isValidHeight,
                                            cv::Mat_<float> &depth)
{
    if( !m_cloudFromSensorXYZRGB)
    {
        ROS_ERROR_STREAM("Point Cloud not set");
        return ;
    }
    height3Dresult = cv::Mat_<float>(m_cloudFromSensorXYZRGB->height,m_cloudFromSensorXYZRGB->width);
    depth = cv::Mat_<float>(m_cloudFromSensorXYZRGB->height,m_cloudFromSensorXYZRGB->width);
    isValidHeight = cv::Mat(m_cloudFromSensorXYZRGB->height,m_cloudFromSensorXYZRGB->width,CV_8UC1);
    std::cout<<" h/w"<< m_cloudFromSensorXYZRGB->height<<m_cloudFromSensorXYZRGB->width<<std::endl;
    for(uint32_t y = 0; y < m_cloudFromSensorXYZRGB->height ; y++)
    {
        for(uint32_t x=0;x < m_cloudFromSensorXYZRGB->width ; x++)
        {
            depth(y,x) = m_cloudFromSensorXYZRGB->points[ y * m_cloudFromSensorXYZRGB->width + x ].z;
            if (std::isfinite(m_cloudFromSensorXYZRGB->points[ y * m_cloudFromSensorXYZRGB->width + x ].z) )
            {
                isValidHeight.at<uchar>(y,x) = 1;

                height3Dresult(y,x) = m_cloudFromSensorXYZRGB->points[ y * m_cloudFromSensorXYZRGB->width + x ].z;
            }
            else
            {
                isValidHeight.at<uchar>(y,x)= 0;
                height3Dresult(y,x) = NAN;
            }
        }
    }
}


void ApcToteFeatures::computeDistance2WallFeature(cv::Mat_<float>& result)
{
    // check if Pointcloud,cameramodel,box are set
    if( !m_cloudFromSensorXYZRGB )
    {
        ROS_ERROR_STREAM("Point Cloud not set");
    }

    // get the tote min/max values from apc_shelf_model/dimensions.cpp
    std::vector<float> toteCoordinates = apc_shelf_model::getToteCoordinates();

    // extract zchannel for computeRelativeLocalMinFeature
    zChannel = cv::Mat_<float>(m_cloudFromSensorXYZRGB->height,m_cloudFromSensorXYZRGB->width);
    m_isValid3Dmask = cv::Mat_<uint8_t>(m_cloudFromSensorXYZRGB->height,m_cloudFromSensorXYZRGB->width);
    result      =    cv::Mat_<float> (m_cloudFromSensorXYZRGB->height,m_cloudFromSensorXYZRGB->width);

    for(size_t t = 0;t < m_cloudFromSensorXYZRGB->points.size();t++)
    {
        auto& p = m_cloudFromSensorXYZRGB->points[t];
        if (p.z < 0 || !std::isfinite(p.z))
        {
            zChannel(t/m_cloudFromSensorXYZRGB->width, t%m_cloudFromSensorXYZRGB->width) =  0;
            m_isValid3Dmask(t/m_cloudFromSensorXYZRGB->width, t%m_cloudFromSensorXYZRGB->width) = 0;
        }
        else
        {
            zChannel(t/m_cloudFromSensorXYZRGB->width, t%m_cloudFromSensorXYZRGB->width) =  std::min( p.z, 0.2f );
            m_isValid3Dmask(t/m_cloudFromSensorXYZRGB->width, t%m_cloudFromSensorXYZRGB->width) = 1;
        }
        float min_dist = std::min({

                                          p.x - ( -1 * toteCoordinates[0]),
                                          toteCoordinates[0] - p.x,

                                          p.y - ( -1 * toteCoordinates[1]),
                                          toteCoordinates[1] - p.y,

                                          p.z
                                  });

        // Cap to zero (or almost zero to allow for errors)
        min_dist = std::max(min_dist, -0.01f);

        result(t/m_cloudFromSensorXYZRGB->width, t%m_cloudFromSensorXYZRGB->width) = min_dist ;

    }
}


void ApcToteFeatures::computeRelativeMeanDiffFeature(cv::Mat_<float>& result)
{
    double min,max;
    cv::Point minLoc,maxLoc;
    cv::minMaxLoc(zChannel,&min,&max,&minLoc,&maxLoc);
    std::cout<<" min/max "<< min<<"  " << max <<std::endl;
    if( zChannel.empty() )
    {
        ROS_ERROR_STREAM("zChannel is empty! call computeDistance2wallFeature() first");
    }

    cv::Mat_<float> zChannelMedian(zChannel.rows,zChannel.cols);

    zChannelMedian = medianFilter(zChannel, 81);

    cv::minMaxLoc(zChannelMedian,&min,&max,&minLoc,&maxLoc);
    std::cout<<" zChannelMedian min/max "<< min<<"  " << max <<std::endl;

//    cv::imshow("median",zChannelMedian * 3);
//    cv::waitKey(0);

    result = zChannel - zChannelMedian;

    result += 0.2; // scale the result by the height of the tote to make sure we hv no negative values

    result /= 0.4;

    result.setTo(NAN,m_isValid3Dmask == 0);



}

void ApcToteFeatures::computeEdgeFeature(cv::Mat& EdgeOut)
{
    EdgeOut  = cv::Mat(m_cloudFromSensorXYZRGB->height,m_cloudFromSensorXYZRGB->width, CV_8UC1);
    cv::Mat rgbImage( m_cloudFromSensorXYZRGB->height,m_cloudFromSensorXYZRGB->width, CV_8UC3);

    //generate rgb image from pointcloud
    for(uint32_t y = 0; y < m_cloudFromSensorXYZRGB->height ; y++)
    {
        for(uint32_t x=0;x < m_cloudFromSensorXYZRGB->width ; x++)
        {
            rgbImage.at<cv::Vec3b>(y,x) = cv::Vec3b(m_cloudFromSensorXYZRGB->points[ y * m_cloudFromSensorXYZRGB->width + x ].r,
                                                    m_cloudFromSensorXYZRGB->points[ y * m_cloudFromSensorXYZRGB->width + x ].g,
                                                    m_cloudFromSensorXYZRGB->points[ y * m_cloudFromSensorXYZRGB->width + x ].b);
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

void ApcToteFeatures::setRGBImage(cv::Mat& in)
{
    this->rgbImage = in;

}
void ApcToteFeatures::setCameraModel(const sensor_msgs::CameraInfoConstPtr& msg)
{
    m_cam_model.fromCameraInfo(msg);
}

cv::Mat ApcToteFeatures::computeHHA(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData, const pcl::PointCloud<pcl::Normal>::ConstPtr& normalCloud)
{
    cv::Mat_<cv::Vec3b> result(m_cloudFromSensorXYZRGB->height, m_cloudFromSensorXYZRGB->width);

    for(unsigned int y = 0; y < m_cloudFromSensorXYZRGB->height; ++y)
    {
        for(unsigned int x = 0; x < m_cloudFromSensorXYZRGB->width; ++x)
        {
            auto& point = (*m_cloudFromSensorXYZRGB)(x,y);
            auto& rawPoint = (*rawData)(x,y);
            auto& normalPoint = (*normalCloud)(x,y);

            if(!pcl::isFinite(point) || !pcl::isFinite(normalPoint))
                result(y,x) = cv::Vec3b(0, 0, 0);
            else
            {
                const float MIN_DEPTH = 0.2;
                const float MAX_DEPTH = 0.7;
                const float MAX_HEIGHT = 0.4;

                float disparity = (255.0f / (MAX_DEPTH/MIN_DEPTH)) * (MAX_DEPTH / std::max(rawPoint.z, MIN_DEPTH));
                float height = (255.0f / MAX_HEIGHT) * (point.z);

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

} // end namespace apc_object_preception
