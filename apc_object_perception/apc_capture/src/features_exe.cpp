/*
 * This file will be generated as an executable and wrapper for apc_features
 * Takes a ***folder*** within ***resource folder***  pcd file as input and calls apc_features
 * The PCD is located in the packagepath/resources/pcd and file name is given as command line argument
 * Hv to think abt multiple file interface, a dir iterator is more meaningful
 */

#include <ros/package.h>
#include <apc_capture/apc_features.h>
#include <apc_capture/box_registration.h>
#include <apc_objects/apc_objects.h>
#include <boost/filesystem.hpp>
#include <camera_calibration_parsers/parse_ini.h>
#include <sstream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/algorithm/string/split.hpp>
#include <string>
#include <pcl/filters/conditional_removal.h>


namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
	if(argc != 2)
	{
		fprintf(stderr, "Usage: apc_features_exe <path to dataset>\n");
		return 1;
	}

	std::string rosPackagePath = ros::package::getPath("apc_capture");
	std::string dataPath = argv[1];

	fs::path dataDir(dataPath);
	if(!fs::is_directory(dataDir))
	{
		fprintf(stderr, "Could not open the data directory.\n");
		return 0;
	}

	// Gather list of images
	std::vector<fs::path> jobs;
	{
		auto it = fs::recursive_directory_iterator(dataDir);
		auto end = fs::recursive_directory_iterator();

		for(; it != end; ++it)
		{
			fs::path path = *it;
			if(fs::is_directory(path) && fs::exists(path / "polygons.yaml"))
				jobs.push_back(fs::canonical(path));
		}
	}

	std::string iniFileName, pcdFileName, boxIndexFileName, rgbFileName, shelfMaskFileName;

	apc_object_perception::ApcFeatures features;

	// Load shelf mesh
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudfromMesh (new pcl::PointCloud<pcl::PointXYZ>);
	std::string MeshPath = rosPackagePath+"/resource/pcd/shelf_withbackface.pcd";
	if(pcl::io::loadPCDFile(MeshPath, *m_cloudfromMesh) < 0)
	{
		fprintf(stderr, "Could not load shelf mesh\n");
		return 1;
	}

	// Translate to distRobot2shelf  (FIXME: This is a magic number)
	Eigen::Affine3f translation(Eigen::Translation3f(apc_objects::distRobot2Shelf,0,0));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*m_cloudfromMesh,*cloudTemp,translation.matrix());
	m_cloudfromMesh=cloudTemp;

	features.setMeshCloud(m_cloudfromMesh);

	for(const fs::path& path : jobs)
	{
		printf("\n\nProcessing frame %s\n", path.c_str());

		iniFileName = (path / "camera.ini").string();
		pcdFileName = (path / "frame.pcd").string();
		rgbFileName = (path / "rgb.png").string();
		boxIndexFileName = (path / "box_index.txt").string();
		shelfMaskFileName = (path / "mask_container.png").string();

		std::ifstream File(boxIndexFileName);
		std::string line;
		Eigen::Vector2i box_indices;
		std::vector<std::string> indices;

		while (std::getline(File, line)) {
			boost::algorithm::split(indices, line, boost::algorithm::is_any_of(" "));
			std::string::size_type sz;

			box_indices(0) = std::stoi(indices[0], &sz);
			box_indices(1) = std::stoi(indices[1], &sz);

			std::cout << "Box indices: " << box_indices(0)<< " " << box_indices(1) << std::endl;
		}

		features.setBoxIndices(box_indices);
		apc_capture::BoxCoordinates boxMsg;
		boxMsg = features.getBoxCoordinates();

		Eigen::Vector3f cameraGuess;
		cameraGuess.x() = apc_objects::distRobot2Shelf + boxMsg.boxCoordinates[0] - 0.4f; // 20 cms infont of the box front face
		cameraGuess.y() = ( boxMsg.boxCoordinates[3] + boxMsg.boxCoordinates[2] ) / 2;
		cameraGuess.z() = ( boxMsg.boxCoordinates[4] + boxMsg.boxCoordinates[5] ) / 2 + 0.04f;

		std::cout << "camera guess: " << cameraGuess.transpose() << std::endl;

		// Camera
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudfromSensorXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloudfromSensorXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
		if(pcl::io::loadPCDFile(pcdFileName, *m_cloudfromSensorXYZRGB) < 0)
		{
			fprintf(stderr, "Could not load PCD file 'frame.pcd' for image '%s'\n", path.c_str());
			return 1;
		}
		pcl::copyPointCloud(*m_cloudfromSensorXYZRGB, *m_cloudfromSensorXYZ);

		cv::Mat rgbImage = cv::imread(rgbFileName);
		cv::Mat shelfMask = cv::imread(shelfMaskFileName,CV_LOAD_IMAGE_GRAYSCALE);

		// Erode the shelf mask (we want to discard points rather than use them for ICP)
		{
			constexpr int erosion_size = 10;
			cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
				cv::Size(2*erosion_size + 1, 2*erosion_size+1),
				cv::Point(erosion_size, erosion_size)
			);
			cv::erode(shelfMask, shelfMask, element);
		}

		Eigen::Affine3f translationCaminWorld;
		translationCaminWorld = Eigen::Translation3f(cameraGuess);
		Eigen::Affine3f transformCaminWorld = translationCaminWorld *
				Eigen::AngleAxisf(10 *(M_PI/180) ,Eigen::Vector3f::UnitY()) *
				Eigen::AngleAxisf(M_PI/2 ,Eigen::Vector3f::UnitY()) *
				Eigen::AngleAxisf(M_PI/2 ,Eigen::Vector3f::UnitZ()) ;

		printf("Running ICP...\n");
		Eigen::Affine3f camPoints2Shelf = features.doRegistration(m_cloudfromSensorXYZ, transformCaminWorld, shelfMask);
		printf("done.\n");

		// Transform sensor cloud into shelf frame
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInShelfFrame (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*m_cloudfromSensorXYZRGB,*cloudInShelfFrame,camPoints2Shelf);

		// Filter out some outliers
		float error_limit = 0.05f;// *** check this***
		pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::GT, boxMsg.boxCoordinates[0] - error_limit)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("x", pcl::ComparisonOps::LT, boxMsg.boxCoordinates[1] + error_limit)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::GT, boxMsg.boxCoordinates[2] - error_limit)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, boxMsg.boxCoordinates[3] + error_limit)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, boxMsg.boxCoordinates[4] - error_limit))); //0.08
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, boxMsg.boxCoordinates[5] + error_limit))); //0.05


		pcl::ConditionalRemoval<pcl::PointXYZRGB> removal (range_cond);
		removal.setInputCloud (cloudInShelfFrame);
		removal.setKeepOrganized(true);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudT (new pcl::PointCloud<pcl::PointXYZRGB>);
		removal.filter (*cloudT);
		cloudInShelfFrame = cloudT;

		std::string cameraName;
		sensor_msgs::CameraInfoPtr camInfo(new sensor_msgs::CameraInfo);
		camera_calibration_parsers::readCalibrationIni(iniFileName,cameraName,*camInfo);
		features.setPointCloud(cloudInShelfFrame);
		features.setCameraModel(camInfo);
		features.setRGBImage(rgbImage);

		cv::Mat_<float> height3D,height2D,dist2shelf,depth;
		cv::Mat isValidHeight;
		cv::Mat edgeFeature;
		
		Eigen::Affine3f shelfToCameraFrame = camPoints2Shelf.inverse();

		cv::Mat binMask;
		features.createBinMask(shelfToCameraFrame,binMask);
		features.computeDist2ShelfFeature(dist2shelf);
		features.computeHeightFeatures(camPoints2Shelf,height3D,height2D,isValidHeight,depth);
		features.computeEdge(edgeFeature);

		cv::Mat hha = features.computeHHA(camPoints2Shelf, m_cloudfromSensorXYZRGB);

		// New HVA, HHV encodings
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgvCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::io::loadPCDFile((path / "frame_tgv.pcd").string(), *tgvCloud);

			if(tgvCloud->size() > 0)
			{
				cv::Mat_<uint8_t> validMask = cv::imread((path / "frame_tgv_weights.png").string(), CV_LOAD_IMAGE_GRAYSCALE);

				for(int y = 0; y < validMask.rows; ++y)
				{
					for(int x = 0; x < validMask.cols; ++x)
					{
						if(validMask(y,x) != 0)
							validMask(y,x) = 100;
					}
				}

				cv::Mat hva = features.computeHVA(camPoints2Shelf, tgvCloud, validMask);
				cv::Mat hhv = features.computeHHV(camPoints2Shelf, tgvCloud, validMask);
				cv::Mat hhav = features.computeHHAV(camPoints2Shelf, tgvCloud, validMask);

				cv::imwrite((path / "feature_hva.png").string(), hva);
				cv::imwrite((path / "feature_hhv.png").string(), hhv);
				cv::imwrite((path / "feature_hhav.png").string(), hhav);
			}
		}

		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgvCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::io::loadPCDFile((path / "frame_upper_filled.pcd").string(), *tgvCloud);

			if(tgvCloud->size() == 0) // can happen on run frames
				pcl::io::loadPCDFile((path / "frame_tgv.pcd").string(), *tgvCloud);

			if(tgvCloud->size() > 0)
			{
				cv::Mat_<uint8_t> validMask = cv::imread((path / "frame_tgv_weights.png").string(), CV_LOAD_IMAGE_GRAYSCALE);

				for(int y = 0; y < validMask.rows; ++y)
				{
					for(int x = 0; x < validMask.cols; ++x)
					{
						if(validMask(y,x) != 0)
							validMask(y,x) = 100;
					}
				}

				cv::Mat hha = features.computeHHA(camPoints2Shelf, tgvCloud);
				cv::Mat hva = features.computeHVA(camPoints2Shelf, tgvCloud, validMask);
				cv::Mat hhv = features.computeHHV(camPoints2Shelf, tgvCloud, validMask);
				cv::Mat hhav = features.computeHHAV(camPoints2Shelf, tgvCloud, validMask);

				cv::imwrite((path / "feature_upper_hha.png").string(), hha);
				cv::imwrite((path / "feature_upper_hva.png").string(), hva);
				cv::imwrite((path / "feature_upper_hhv.png").string(), hhv);
				cv::imwrite((path / "feature_upper_hhav.png").string(), hhav);
			}
		}

		// For visualizing
		cv::Mat height2DViz (height2D.rows,height2D.cols,CV_8UC1);
		cv::Mat height3DViz (height2D.rows,height2D.cols,CV_8UC1);
		cv::Mat dist2shelfViz(dist2shelf.rows,dist2shelf.cols,CV_8UC1);
		double min, max;
		double distMin,distMax;
		double height3DMin, height3DMax;

		cv::minMaxLoc(dist2shelf,&distMin,&distMax);
		cv::minMaxLoc(height2D,&min,&max);
		cv::minMaxLoc(height3D, &height3DMin, &height3DMax);
		std::cout<<" Dist2Shelf Min/Max : "<<distMin<<" "<<distMax<<std::endl;

		std::cout << "Height 3D Min/Max: " << height3DMin <<"  "<< height3DMax << "\n";
		height3DMin = -0.02;

		distMin = 0, distMax = 0.1;
		for (int y = 0;y < height2D.rows;y++)
		{
			for (int x = 0; x < height2D.cols;x++)
			{
				height2DViz.at<uchar>(y,x) =cv::saturate_cast<uint8_t>(( ( height2D(y,x) -min ) / ( max - min ) )*255);
				dist2shelfViz.at<uchar>(y,x) =cv::saturate_cast<uint8_t>(( ( dist2shelf(y,x) -distMin ) / ( distMax - distMin ) )*255);
				height3DViz.at<uchar>(y,x) = cv::saturate_cast<uint8_t>( (height3D(y,x) - height3DMin) / (height3DMax - height3DMin) * 255 );
			}
		}

		cv::flip(height2DViz, height2DViz, -1);
		cv::flip(dist2shelfViz, dist2shelfViz, -1);
		cv::flip(height3DViz, height3DViz, -1);

		isValidHeight = 255*isValidHeight ;
		cv::imwrite((path / "feature_height2D.png").string(), height2DViz);
		cv::imwrite((path / "feature_height3D.png").string(), height3DViz);
		cv::imwrite((path / "feature_isValidHeight.png").string(), isValidHeight);
		cv::imwrite((path / "feature_edge.png").string(), edgeFeature);
		cv::imwrite((path / "feature_dist2shelf.png").string(), dist2shelfViz);
		cv::imwrite((path / "feature_hha.png").string(), hha);
		cv::imwrite((path / "bin_mask_estimated.png").string(), binMask);


		FILE* file = fopen((path / "feature_height2D.raw").c_str(), "w");
		fwrite(height2D.data, sizeof(float), height2D.rows*height2D.cols, file);
		fclose(file);

		file = fopen((path / "feature_dist2shelf.raw").c_str(), "w");
		fwrite(dist2shelf.data, sizeof(float), dist2shelf.rows*dist2shelf.cols, file);
		fclose(file);

		file = fopen((path / "feature_height3D.raw").c_str(), "w");
		fwrite(height3D.data, sizeof(float), height3D.rows*height3D.cols, file);
		fclose(file);

		file = fopen((path / "feature_depth.raw").c_str(), "w");
		fwrite(depth.data, sizeof(float), depth.rows*depth.cols, file);
		fclose(file);
	}

	return 0;
}
