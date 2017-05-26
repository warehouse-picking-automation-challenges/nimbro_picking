#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <camera_calibration_parsers/parse_ini.h>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/algorithm/string/split.hpp>
#include <string>
#include <pcl/filters/conditional_removal.h>
#include <apc_capture/apc_tote_features.h>

#include <pcl/features/integral_image_normal.h>

namespace fs = boost::filesystem;
typedef pcl::PointCloud<pcl::PointXYZ> pointCloundXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> pointCloundXYZRGB;

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: apc_tote_features_exe <path to dataset>\n");
        return 1;
    }

    // for now using just one pcd file, later will be chaged to run over entire dataset
    std::string dataPath = argv[1];
    std::string rosPackagePath = ros::package::getPath("apc_capture");

    fs::path dataDir (dataPath);
    if(!fs::is_directory(dataDir) || !fs::is_directory(dataDir))
    {
        fprintf(stderr, "Could not open the data directory.\n");
        return 0;
    }
    std::string iniFileName, pcdFileName, rgbFileName ;

    // load tote mesh
    pointCloundXYZ::Ptr m_cloudfromMesh(new pointCloundXYZ);
    pcl::PointXYZRGB min, max;
    pcl::PointXYZ minXYZ, maxXYZ;
    std::string MeshPath = rosPackagePath + "/resource/pcd/tote_face_removed.pcd";
    if (pcl::io::loadPCDFile(MeshPath, *m_cloudfromMesh) < 0)
    {
        fprintf(stderr, "Could not load tote mesh\n");
        return 1;
    }
    pcl::getMinMax3D(*m_cloudfromMesh, minXYZ, maxXYZ);

    std::cout << " Mesh Min/Max " << minXYZ << " " << maxXYZ << std::endl;

    fs::directory_iterator itr(dataDir);
    fs::directory_iterator end_itr;

    for(;itr != end_itr;itr++) {
        printf("\n\nProcessing frame %s\n", itr->path().c_str());

        iniFileName = itr->path().string() + "/camera.ini";
        pcdFileName = itr->path().string() + "/frame.pcd";
        rgbFileName = itr->path().string() + "/rgb.png";

        std::cout << pcdFileName << std::endl;

        pointCloundXYZ::Ptr m_cloudFromSensorXYZ(new pointCloundXYZ);  //for registration
        pointCloundXYZRGB::Ptr m_cloudFromSensorXYZRGB(new pointCloundXYZRGB);; //for feature computation

        //load rgb image
        cv::Mat rgbImage ;
        rgbImage = cv::imread(rgbFileName);

        // load pcd from file
        std::cout << " trying to load" << std::endl;
        if (pcl::io::loadPCDFile(pcdFileName, *m_cloudFromSensorXYZRGB) < 0) {
            fprintf(stderr, "Could not load PCD file 'frame.pcd' ");
            return 1;
        }
        std::cout << " pcd loaded" << std::endl;
        pcl::copyPointCloud(*m_cloudFromSensorXYZRGB, *m_cloudFromSensorXYZ);

        apc_object_perception::ApcToteFeatures features;

        features.setMeshCloud(m_cloudfromMesh);
        features.setRGBImage(rgbImage);
        std::cout << " Mesh cloud set" << std::endl;
        Eigen::Vector3f cameraGuess;
        cameraGuess.x() = 0;//0.07;
        cameraGuess.y() = 0;//0;
        cameraGuess.z() = 0.7;//0.7;
        std::cout << "camera guess: " << cameraGuess.transpose() << std::endl;

        Eigen::Affine3f translationCaminWorld;
        translationCaminWorld = Eigen::Translation3f(cameraGuess);
        Eigen::Affine3f transformCaminWorld =  translationCaminWorld  * Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitX())  /*  ; Eigen::AngleAxisf(M_PI ,Eigen::Vector3f::UnitZ()) */ ;

        printf("Running ICP...\n");
        Eigen::Affine3f camPoints2Tote = features.doRegistration(m_cloudFromSensorXYZ, transformCaminWorld);
        printf("done.\n");

        std::cout << " camPoints2Tote " << camPoints2Tote.matrix() << std::endl;
        // transform the points in box frame

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInBoxFrame(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*m_cloudFromSensorXYZRGB, *cloudInBoxFrame, camPoints2Tote);

        // Filter out some outliers
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::GT, -0.31)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, 0.31)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::GT, -0.19)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZRGB>("y", pcl::ComparisonOps::LT, 0.19)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 0.4)));


        pcl::ConditionalRemoval<pcl::PointXYZRGB> removal(range_cond);
        removal.setInputCloud(cloudInBoxFrame);
        removal.setKeepOrganized(true);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZRGB>);
        removal.filter(*cloudT);
        cloudInBoxFrame = cloudT;

        std::string cameraName;
        sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo);
        camera_calibration_parsers::readCalibrationIni(iniFileName,cameraName,*camInfo);


        pcl::getMinMax3D(*cloudInBoxFrame, min, max);

        std::cout << " Min/Max " << min << " " << max << std::endl;

          // compute the features
        cv::Mat_<float> height3D, depth, dist2wall, relativeMin;
        cv::Mat isValidHeight;
        cv::Mat edgeFeature;
        features.setPointCloud(cloudInBoxFrame);
        features.setCameraModel(camInfo);
        features.computeHeightFeatures(height3D, isValidHeight, depth);
        features.computeEdgeFeature(edgeFeature);
        features.computeDistance2WallFeature(dist2wall);
        features.computeRelativeMeanDiffFeature(relativeMin);
        cv::Mat toteMask ;

        Eigen::Affine3f toteToCameraFrame = camPoints2Tote.inverse();
        features.createToteMask(toteToCameraFrame,toteMask);

		pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
		{
			pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

			ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
			ne.setMaxDepthChangeFactor(0.8f);
			ne.setNormalSmoothingSize(30.0f);

			ne.setInputCloud(m_cloudFromSensorXYZRGB);
			ne.compute(*normalCloud);
		}

        cv::Mat hha = features.computeHHA(camPoints2Tote, m_cloudFromSensorXYZRGB, normalCloud) ;

        // For visualizing
        cv::Mat height3DViz(height3D.rows, height3D.cols, CV_8UC1);
        cv::Mat dist2wallViz(dist2wall.rows, dist2wall.cols, CV_8UC1);
        cv::Mat relativeMinViz(relativeMin.rows, relativeMin.cols, CV_8UC1);

        float height3DMax = 0.4f;
        float dist2wallMax = 0.2f;

/*      double height3DMin, height3DMax;
        cv::minMaxLoc(height3D, &height3DMin, &height3DMax);
        std::cout << "Height 3D Min/Max: " << height3DMin << height3DMax << "\n";*/

        for (int y = 0; y < height3D.rows; y++) {
            for (int x = 0; x < height3D.cols; x++) {
                height3DViz.at<uchar>(y, x) = cv::saturate_cast<uint8_t>( ( ( height3D(y, x)  ) / height3DMax  ) * 255);
                dist2wallViz.at<uint8_t>(y,x) = cv::saturate_cast<uint8_t>( ( ( dist2wall(y, x)  ) / dist2wallMax  ) * 255);
                relativeMinViz.at<uint8_t>(y,x) = cv::saturate_cast<uint8_t>( (  relativeMin(y, x)    ) * 255);
            }
        }

        cv::flip(height3DViz, height3DViz, -1);

      double min, max;
        cv::minMaxLoc(height3D, &min, &max);
        std::cout << "Height 3D Min/Max: " << min << " " << max << "\n";


        isValidHeight = 255 * isValidHeight;
        cv::imwrite(itr->path().string() + "/feature_height3D.png", height3DViz);
        cv::imwrite(itr->path().string() + "/feature_isValidHeight.png", isValidHeight);
        cv::imwrite(itr->path().string() + "/feature_edge.png", edgeFeature);
        cv::imwrite(itr->path().string() + "/feature_dist2wall.png", dist2wallViz);
        cv::imwrite(itr->path().string() + "/feature_relativeMeanDiff.png", relativeMinViz);
        cv::imwrite((itr->path() / "bin_mask_estimated.png").c_str(),toteMask);
        cv::imwrite((itr->path() / "feature_hha.png").c_str(), hha);


        FILE *file = fopen((itr->path().string() + "/feature_height3D.raw").c_str(), "w");
        fwrite(height3D.data, sizeof(float), height3D.rows * height3D.cols, file);
        fclose(file);

        file = fopen((itr->path().string() + "/feature_depth.raw").c_str(), "w");
        fwrite(depth.data, sizeof(float), depth.rows * depth.cols, file);
        fclose(file);

        file = fopen((itr->path().string() + "/feature_dist2wall.raw").c_str(), "w");
        fwrite(dist2wall.data, sizeof(float), dist2wall.rows * dist2wall.cols, file);
        fclose(file);

        file = fopen((itr->path().string() + "/feature_relativeMeanDiff.raw").c_str(), "w");
        fwrite(relativeMin.data, sizeof(float), relativeMin.rows * relativeMin.cols, file);
        fclose(file);



    }

    std::cout<<" Done "<<std::endl;
    return 0;
}
