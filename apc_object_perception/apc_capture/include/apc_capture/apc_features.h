// Compute features for ***shelf*** Perception from the RGB-D pointcloud
// Author: Arul Selvam Periyasamy <arulselvam@uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef APC_OBJECT_PERCEPTION_FEATURES_H
#define APC_OBJECT_PERCEPTION_FEATURES_H

#include <apc_capture/BoxCoordinates.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>

// PCL includes (disable some warnings for those)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#pragma GCC diagnostic pop

namespace apc_object_perception
{

class ApcFeatures
{
public:
	ApcFeatures();
	virtual ~ApcFeatures();

	//! @name Input data
	//@{
	void setMeshCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& meshCloud);

	void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);  // only for feature computation (for registration, point cloud (XYZ) is passed as argument)
	void setCameraModel(const sensor_msgs::CameraInfoConstPtr&);
	void setBoxIndices(const Eigen::Vector2i& indices);
	void setBoxCoordinates(apc_capture::BoxCoordinates&);
	apc_capture::BoxCoordinates getBoxCoordinates();
	//@}

	/**
	 * @brief Register sensor point cloud against the shelf
	 *
	 * @param[in] cloudFromSensor Sensor point cloud
	 * @param[in] transformSensorInWorldFrame Guessed sensor position in world frame
	 * @param[in] binary shelf mask represting the pixels belonging to the shelf (255 - belongs to the shelf / 0 - belongs to the objects)
	 * @param[out] correctedShelf Shelf position in world frame (optional)
	 * @return Sensor position in world frame
	 **/
	Eigen::Affine3f doRegistration(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloudFromSensor,
		const Eigen::Affine3f& transformSensorInWorldFrame,
		const cv::Mat_<uint8_t>& shelfMask,
		Eigen::Affine3f* correctedShelf = 0
	);

	inline pcl::PointCloud<pcl::PointXYZ>::ConstPtr filteredCloud() const
	{ return m_filteredCloud; }

	inline pcl::PointCloud<pcl::PointXYZ>::ConstPtr filteredMeshCloud() const
	{ return m_filteredMeshCloud; }

	inline pcl::PointCloud<pcl::PointXYZ>::ConstPtr transformedCloud() const
	{ return m_transformedCloud; }

	/**
	 * @brief create BinMask using the camera model and the box coordinates computed. The coordinates of the front face of the bin is considered
	 *
	 * @param[in] transformation from shelfFrame to cameraFrame ( inverse of that doRegistration() result)
	 *  @param[out] computed binMask
	 **/
	void createBinMask(Eigen::Affine3f,cv::Mat&);
	
	

	void computeDist2ShelfFeature(cv::Mat_<float>& result);
	//-------------------------START--------------------- combined into one function ***computeHeightFeatures()***
	void computeMissingHeight(Eigen::Affine3f& tfTransIn , cv::Mat_<float>& result);
	void computeHeight3D(cv::Mat_<float>& result);
	void computeIsValid3D(cv::Mat& result);
	//-------------------------END------------------------
	void computeEdge(cv::Mat& EdgeOut);


	/**
	 * @brief compute the height related features
	 *
	 * @param[in] tfTransIn translation of the camera's position in shelf frame (used for getting the camera ray
	 * @param[out] height3Dresult Hieght of each pixel w.r.to box gound plane  (called as height3D in RBO)
	 * @param[out] MissingHeightappromation  computed missing 3D heights (called as height2D in RBO)
	 * @param[out] isValidHeight binary values indicating if depth data is present (true) or not (false)
	 * @param[out] depth	binary values indicating if a pixel is in the vicinity of an edge
	 * @return void
	 **/
	void computeHeightFeatures(
		Eigen::Affine3f& tfTransIn,cv::Mat_<float>& height3Dresult,
		cv::Mat_<float>& MissingHeightappromation, cv::Mat& isValidHeight,
		cv::Mat_<float>& depth
	);

	cv::Mat computeHHA(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData);
	cv::Mat computeHHAV(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData, const cv::Mat_<uint8_t>& mask);
	cv::Mat computeHVA(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData, const cv::Mat_<uint8_t>& mask);
	cv::Mat computeHHV(const Eigen::Affine3f& cameraPose, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& rawData, const cv::Mat_<uint8_t>& mask);
	void setRGBImage(cv::Mat& in);

private:
	void computeBoxCoordinates();

	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr m_cloudfromSensorXYZRGB;
	image_geometry::PinholeCameraModel m_cam_model;
	Eigen::Vector2i m_boxIndices;

	double m_boxPlaneHeight;
	apc_capture::BoxCoordinates m_boxmsg;
	cv::Mat rgbImage;


	// for ICP
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_meshCloud;
// 	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> m_icp;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr m_filteredCloud;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr m_filteredMeshCloud;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr m_transformedCloud;

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> m_icp;


	bool m_isBoxIndicesSet = false;
	const float m_leafSize = 0.008f;
};


} //end namspace

#endif
