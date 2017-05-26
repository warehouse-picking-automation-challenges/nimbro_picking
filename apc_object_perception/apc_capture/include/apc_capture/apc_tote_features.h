// Compute features for ***tote*** Perception from the RGB-D pointcloud
// Author: Arul Selvam Periyasamy <arulselvam@uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>



#ifndef APC_OBJECT_PERCEPTION_TOTE_FEATURES_H
#define APC_OBJECT_PERCEPTION_TOTE_FEATURES_H
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
// PCL includes (disable some warnings for those)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>
#pragma GCC diagnostic pop

namespace apc_object_perception
{

class ApcToteFeatures
{
public:
    ApcToteFeatures();
    virtual ~ApcToteFeatures();


    //! @name Input data
    //@{
    void setMeshCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& meshCloud);

    void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
    void setCameraModel(const sensor_msgs::CameraInfoConstPtr&);
    //@}

    /**
	 * @brief Register sensor point cloud against the tote
	 *
	 * @param[in] cloudFromSensor Sensor point cloud
	 * @param[in] transformSensorInWorldFrame Guessed sensor position in world frame
	 * @param[out] correctedShelf Shelf position in world frame (optional)
	 * @return Sensor position in world frame
	 **/
    Eigen::Affine3f doRegistration(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudFromSensor,
            const Eigen::Affine3f& transformSensorInWorldFrame,
            Eigen::Affine3f* correctedShelf = 0
    );

    /**
	 * @brief create toteMask using the camera model and the tote coordinates. The coordinates of the top face of the tote is considered
	 *
	 * @param[in] transformation from tote to cameraFrame ( inverse of that doRegistration() result)
	 *  @param[out] computed toteMask
	 **/
    void createToteMask(Eigen::Affine3f,cv::Mat&);

    /**
	 * @brief compute the height related features
	 *
	 * @param[out] height3Dresult   Hieght of each pixel w.r.to box gound plane  (called as height3D in RBO)
	 * @param[out] isValidHeight    binary values indicating if depth data is present (true) or not (false)
	 * @param[out] depth    binary values indicating if a pixel is in the vivinity of an edge
	 * @return void
	 **/
    void computeHeightFeatures(
           cv::Mat_<float>& height3Dresult,
           cv::Mat& isValidHeight,
           cv::Mat_<float>& depth
    );

    void computeEdgeFeature(cv::Mat& EdgeOut);
    cv::Mat computeHHA(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData, const pcl::PointCloud<pcl::Normal>::ConstPtr& normalCloud);
    void setRGBImage(cv::Mat& in);
    void computeDistance2WallFeature(cv::Mat_<float>& result);
    // computeRelativeLocalMinFeature should always be called after computeDistance2WallFeature
    void computeRelativeMeanDiffFeature(cv::Mat_<float>& result);


private:

    typedef  pcl::PointCloud<pcl::PointXYZRGB> pointCloudXYZRGB;
    typedef  pcl::PointCloud<pcl::PointXYZ>    pointCloudXYZ;
    image_geometry::PinholeCameraModel m_cam_model;

    // for feature computation
    pointCloudXYZRGB::Ptr m_cloudFromSensorXYZRGB;
    cv::Mat_<uint8_t> m_isValid3Dmask;

    // for filtering box pixels - ICP preprocessing
    cv::Mat rgbImage;
    cv::Mat yuvImage;

    // for ICP
    pointCloudXYZ::Ptr m_meshCloud;
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> m_icp;
    cv::Mat_<float> zChannel;


};

}//end namspace

#endif // APC_OBJECT_PERCEPTION_TOTE_FEATURES_H
