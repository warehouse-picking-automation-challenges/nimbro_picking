#include <apc_6dpose/apc_6dpose.h>

#include <ros/console.h>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/registration/gicp6d.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <iterator>
#include <Eigen/Geometry>

#include <boost/any.hpp>
#include <boost/program_options.hpp>
#include <fstream>

#ifndef APC_6DPOSE_MOCK

#include <v4r/common/visibility_reasoning.h>
#include <v4r/common/miscellaneous.h>  // to extract Pose intrinsically stored in pcd file
#include <v4r/io/filesystem.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>
#include <v4r/common/visibility_reasoning.h>
#include <v4r/recognition/local_recognizer.h>
#include <v4r/recognition/registered_views_source.h>

#endif

namespace apc_6dpose
{

void readGraspingPositions(const std::string filename, pcl::PointCloud<pcl::PointXYZ> &grasp_points, pcl::PointCloud<pcl::PointXYZ> &grasp_directions){
    std::ifstream grasp_file(filename);
    std::string line;
    while (std::getline(grasp_file, line)) {
        if (boost::starts_with(line, "#"))
            continue;
        std::stringstream lineStream(line);
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
            grasp_points.points.push_back(position);
            grasp_directions.points.push_back(direction);
        }
    }
}

class APC6DPosePrivate
{
public:
#ifndef APC_6DPOSE_MOCK
    typedef pcl::PointXYZRGB PointT;
    typedef v4r::Model<PointT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;
    boost::shared_ptr<v4r::MultiRecognitionPipeline<PointT> > m_recognizer;
    double m_confidence_threshold;
    double m_chopZ;
    bool m_filter;
    bool m_use_naive_approach;
    bool m_use_visibility_reasoning;
    int m_icp_type;
    bool m_perform_dilate;

    std::vector<ModelTPtr> m_verified_models;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > m_verified_transformations;
    pcl::PointCloud<pcl::PointXYZ> m_grasping_points;
    pcl::PointCloud<pcl::PointXYZ> m_grasping_directions;
    std::vector<ModelTPtr> m_models;
    boost::shared_ptr <v4r::Source<pcl::PointXYZRGB> > m_cast_source;

    APC6DPosePrivate()
        :m_confidence_threshold(0.5),
        m_chopZ(10),
        m_filter(false),
        m_use_naive_approach(false),
        m_use_visibility_reasoning(false),
        m_icp_type(0),
        m_perform_dilate(true){}
#endif

    cv::Mat_<uint8_t> m_bin_mask;

};

    APC6DPose::APC6DPose(int argc, char* argv[])
     :m_p(new APC6DPosePrivate)

    {
#ifndef APC_6DPOSE_MOCK
        initialize(argc, argv);
#endif
    }

    APC6DPose::APC6DPose()
    :m_p(new APC6DPosePrivate)
    {}


    APC6DPose::~APC6DPose(){}



    void APC6DPose::initialize(int argc, char* argv[]){
#ifndef APC_6DPOSE_MOCK

        if(!m_p->m_use_naive_approach){
            m_p->m_recognizer.reset(new v4r::MultiRecognitionPipeline<PointT>(argc, argv));
        }

        boost::shared_ptr <v4r::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB, pcl::PointXYZRGB> > src
                (new v4r::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB, pcl::PointXYZRGB>(0.01));


        src->setPath (argv[2]);
        src->generate ();
        m_p->m_cast_source = boost::static_pointer_cast<v4r::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB, pcl::PointXYZRGB> > (src);

        boost::shared_ptr<v4r::LocalRecognitionPipeline<pcl::PointXYZRGB> > sift_r;
        m_p->m_models = m_p->m_cast_source->getModels();
        std::cout << "Got " << m_p->m_models.size()<< " models."<< std::endl;
#endif
    }

    void APC6DPose::setICPType(int icp_type){
#ifndef APC_6DPOSE_MOCK
        m_p->m_icp_type = icp_type;
#endif
    }

    void APC6DPose::setPerformDilate(bool perfom_dilate){
#ifndef APC_6DPOSE_MOCK
        m_p->m_perform_dilate = perfom_dilate;
#endif
    }

    void APC6DPose::setUseVisibility(bool use_visibility_reasoning ){
#ifndef APC_6DPOSE_MOCK
        m_p->m_use_visibility_reasoning = use_visibility_reasoning;
#endif
    }

    void APC6DPose::setUseNaiveApproach(bool use_naive_approach){
#ifndef APC_6DPOSE_MOCK
        m_p->m_use_naive_approach = use_naive_approach;
#endif
    }

    void APC6DPose::initialize(std::string models_folder, std::string grasp_file){
#ifndef APC_6DPOSE_MOCK
        int argc = 13;
        char* argv[13];
        argv[0] = ((char*)"program");
        argv[1] = ((char*)"--models_dir");
        argv[2] = ((char *)models_folder.c_str());
        argv[3] = ((char*)"--hv_color_sigma_ab");
        argv[4] = ((char*)"1");
        argv[5] = ((char*)"--hv_color_sigma_l");
        argv[6] = ((char*)"1");
        //argv[7] = ((char*)"--do_shot");
        argv[7] = ((char*)"--hv_inlier_threshold");
        argv[8] = ((char*)"10");
        argv[9] = ((char*)"--max_corr_distance");
        argv[10] = ((char*)"10");
        argv[11] = ((char*)"--hv_occlusion_threshold");
        argv[12] = ((char*)"10");
        readGraspingPositions(grasp_file, m_p->m_grasping_points, m_p->m_grasping_directions);
        initialize(argc, argv);
#endif
    }




    void APC6DPose::setBinMask(const cv::Mat_<uint8_t>& in_bin_mask){
        m_p->m_bin_mask = in_bin_mask;
#ifndef APC_6DPOSE_MOCK
        m_p->m_confidence_threshold = 0.5;
#endif
    }

    APC6DPose::Result APC6DPose::compute(const PointCloud::ConstPtr& inCloud, const std::string& object, const Eigen::Vector3f& hint, const cv::Mat_<uint8_t> &in_mask_box){
#ifndef APC_6DPOSE_MOCK

        cv::Mat mask_box;
        if(m_p->m_perform_dilate){
            int n = -20;
            int an = n > 0 ? n : -n;
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(an*2+1, an*2+1), cv::Point(an, an) );
            cv::dilate(in_mask_box, mask_box, element);
        }else{
            in_mask_box.copyTo(mask_box);
        }
        std::cout << "Output has " << inCloud->points.size () << " points." << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointIndices indices;
        int nCols = mask_box.cols;
        int nRows = mask_box.rows;
        const uint8_t* p;
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
        extract_indices.setInputCloud (inCloud);
        extract_indices.setKeepOrganized (true);
        extract_indices.filter(*cloud);



        return this->compute(cloud);
#else
            Result result;
            return result;
#endif
    }

 APC6DPose::Result APC6DPose::compute(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inCloud){
#ifndef APC_6DPOSE_MOCK

     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::copyPointCloud(*inCloud, *cloud);
     float resolution = 0.02;

    if( m_p->m_chopZ > 0){
        pcl::PassThrough<PointT> pass;
        pass.setFilterLimits ( 0.f, m_p->m_chopZ );
        pass.setFilterFieldName ("z");
        pass.setInputCloud (cloud);
        pass.setKeepOrganized (true);
        pass.filter (*cloud);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudValid(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	filter.filter(*cloudValid);

	if(cloudValid->size() < 25)
	{
		ROS_ERROR("less than 25 points remaining, aborting");
		Result result;
		result.distance = std::numeric_limits<float>::infinity();
		return result;
	}

    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr subsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    uniform_sampling.setRadiusSearch(resolution);
    uniform_sampling.setInputCloud(cloudValid);
    pcl::PointCloud<int> sampled_indices;
    uniform_sampling.compute(sampled_indices);

    pcl::copyPointCloud(*cloudValid, sampled_indices.points , *subsampled_cloud);

	if(subsampled_cloud->size() < 22)
	{
		ROS_ERROR("less than 22 points remaining, aborting");
		Result result;
		result.distance = std::numeric_limits<float>::infinity();
		return result;
	}

    // Computing recognizition
    if(!m_p->m_use_naive_approach) {
        m_p->m_recognizer->setInputCloud (cloud);
        //m_p->m_recognizer->setInputCloud (subsampled_cloud);

        m_p->m_recognizer->recognize();
        m_p->m_verified_models = m_p->m_recognizer->getVerifiedModels();
        m_p->m_verified_transformations = m_p->m_recognizer->getVerifiedTransforms();
        if(m_p->m_verified_models.size() == 0){
            m_p->m_verified_models = m_p->m_recognizer->getModels();
            m_p->m_verified_transformations = m_p->m_recognizer->getTransforms();
        }
    }
    // naive initialization
    if(m_p->m_verified_models.size() == 0 ){

        boost::shared_ptr<v4r::Model<PointT>> m = m_p->m_models[0];
        if(!m_p->m_cast_source->getLoadIntoMemory())
            m_p->m_cast_source->loadInMemorySpecificModel(*m);
        Eigen::Vector4f centroid_model;
        Eigen::Vector4f centroid_scene;
        pcl::PointCloud<PointT>::ConstPtr dense_cloud =  m->getAssembled (resolution);
        pcl::compute3DCentroid(*cloud, centroid_scene);
        pcl::compute3DCentroid(*dense_cloud, centroid_model);
        Eigen::Affine3f t(Eigen::Translation3f(Eigen::Vector3f(centroid_scene[0] - centroid_model[0],
                          centroid_scene[1] - centroid_model[1], centroid_scene[2] - centroid_model[2])));
        Eigen::Matrix4f transform_base = t.matrix();
        m_p->m_verified_models.push_back(m_p->m_models[0]);
        m_p->m_verified_transformations.push_back(transform_base);

        std::vector<float> angles(1);
        angles[0] = M_PI/2.0;
        //always working with one model at a time
        for(int a = 0; a < angles.size(); a++){
            Eigen::Affine3f rotationX = Eigen::Affine3f(Eigen::AngleAxisf(angles[a], Eigen::Vector3f::UnitX()));
            Eigen::Affine3f rotationY = Eigen::Affine3f(Eigen::AngleAxisf(angles[a], Eigen::Vector3f::UnitY()));
            Eigen::Affine3f rotationZ = Eigen::Affine3f(Eigen::AngleAxisf(angles[a], Eigen::Vector3f::UnitZ()));

            Eigen::Matrix4f transformX = t.matrix()*rotationX.matrix();
            Eigen::Matrix4f transformY = t.matrix()*rotationY.matrix();
            Eigen::Matrix4f transformZ = t.matrix()*rotationZ.matrix();

            m_p->m_verified_models.push_back(m_p->m_models[0]);
            m_p->m_verified_transformations.push_back(transformX);
            m_p->m_verified_models.push_back(m_p->m_models[0]);
            m_p->m_verified_transformations.push_back(transformY);
            m_p->m_verified_models.push_back(m_p->m_models[0]);
            m_p->m_verified_transformations.push_back(transformZ);

        }
    }


    //Transformin internal values into a more standard values
    Result best_result;
    //m_p->m_recognizer->getPoseRefinement(m_p->m_verified_models, m_p->m_verified_transformations);
    best_result.distance = 100000000000;
    std::cout << "Will check " << m_p->m_verified_models.size() << "hyps" << std::endl;

    std::vector<Result> results(m_p->m_verified_models.size());
    #pragma omp parallel for
    for(int i = 0 ; i < m_p->m_verified_models.size(); i++){
        std::cout << "Checking model " << i << "/" << m_p->m_verified_models.size()<< std::endl;
        pcl::PointCloud<PointT>::ConstPtr model_cloud = m_p->m_verified_models[i]->getAssembled (resolution);
        pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud = m_p->m_verified_models[i]->getNormalsAssembled (resolution);
        pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr normal_aligned (new pcl::PointCloud<pcl::Normal>);
        pcl::transformPointCloud (*model_cloud, *model_aligned, m_p->m_verified_transformations[i]);
        Eigen::Matrix4f icp_transformation;

        double distance;
        pcl::PointCloud<PointT>::Ptr final(new pcl::PointCloud<PointT> ());
        if(m_p->m_icp_type==1){
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setInputTarget (subsampled_cloud);
            icp.setMaximumIterations (100);
            icp.setInputSource(model_aligned);
            icp.setEuclideanFitnessEpsilon(1e-5);
            icp.setTransformationEpsilon(0.001f * 0.001f);
            icp.align(*final);
            distance= icp.getFitnessScore();
            icp_transformation = icp.getFinalTransformation();
            std::cout << "has converged:" << icp.hasConverged() << " score: " << distance << std::endl;
        /*}else if(m_p->m_icp_type==2) {
            pcl::GeneralizedIterativeClosestPoint6D icp;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gicp6d_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gicp6d_model(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr gicp6d_final(new pcl::PointCloud<pcl::PointXYZRGBA>);


            pcl::copyPointCloud(*subsampled_cloud, *gicp6d_cloud);
            pcl::copyPointCloud(*model_aligned, *gicp6d_model);

            icp.setInputTarget (gicp6d_cloud);
            icp.setMaximumIterations (100);
            icp.setInputSource(gicp6d_model);
            icp.setEuclideanFitnessEpsilon(1e-5);
            icp.setTransformationEpsilon(0.001f * 0.001f);
            icp.align(*gicp6d_final);
            distance= icp.getFitnessScore();
            icp_transformation = icp.getFinalTransformation();
            std::cout << "has converged:" << icp.hasConverged() << " score: " << distance << std::endl;
            pcl::copyPointCloud(*gicp6d_final, *final);*/

        } else{
            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
            icp.setInputTarget (subsampled_cloud);
            icp.setMaximumIterations (100);
            icp.setInputSource(model_aligned);
            icp.setEuclideanFitnessEpsilon(1e-5);
            icp.setTransformationEpsilon(0.001f * 0.001f);
            icp.align(*final);
            distance= icp.getFitnessScore();
            icp_transformation = icp.getFinalTransformation();
            std::cout << "has converged:" << icp.hasConverged() << " score: " << distance << std::endl;
        }



        if(m_p->m_use_visibility_reasoning) {
            //focal length and center of projection
            v4r::VisibilityReasoning<pcl::PointXYZRGB> vr (1380.9, 937.4334*2.0, 545.1564*2.0);
            vr.setThresholdTSS (0.01f);
            v4r::transformNormals(*normal_cloud, *normal_aligned, icp_transformation*m_p->m_verified_transformations[i]);
            vr.computeFSVWithNormals (cloud, final, normal_aligned);
            distance = - 1.f + vr.getFSVUsedPoints() / static_cast<float>(model_aligned->points.size());
        }

        ROS_INFO("Found model with distance %f\n", distance);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*final, centroid);
        results[i].objectPose  = icp_transformation*m_p->m_verified_transformations[i];
        results[i].distance=distance;
        results[i].objectCenter = centroid;
    }

    int best_index = 0;
    for(size_t i = 0; i < results.size(); i++){
        if(best_result.distance > results[i].distance){
            best_result = results[i];
            best_index = i;
        }
    }

    pcl::transformPointCloud (*m_p->m_verified_models[best_index]->getAssembled (resolution), best_result.aligned_model, best_result.objectPose);

	best_result.aligned_model.header = inCloud->header;
    std::cout << "Writing file"  << best_result.aligned_model.points.size() << std::endl;
    //pcl::io::savePCDFileASCII ("final.pcd", best_result.aligned_model);
    std::cout << "Finished writing with distance " << best_result.distance << std::endl;
    Eigen::Affine3f transform(best_result.objectPose);
    pcl::transformPointCloud (m_p->m_grasping_directions, best_result.grasping_directions, Eigen::Affine3f(transform.rotation()));
    pcl::transformPointCloud (m_p->m_grasping_points, best_result.grasping_points, best_result.objectPose);
    best_result.grasping_points.width    = m_p->m_grasping_points.points.size();
    best_result.grasping_points.height   = 1;
    best_result.grasping_points.is_dense = false;
    best_result.grasping_points.points.resize (best_result.grasping_points.width * best_result.grasping_points.height);
    best_result.grasping_directions.width   = m_p->m_grasping_directions.points.size();
    best_result.grasping_directions.height   = 1;
    best_result.grasping_directions.is_dense = false;
    best_result.grasping_directions.points.resize (best_result.grasping_directions.width * best_result.grasping_directions.height);
    return best_result;

#else
    Result result;
    return result;
#endif

    }
}
