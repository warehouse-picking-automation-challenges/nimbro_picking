// resgister box with the input point cloud

#ifndef APC_OBJECT_PERCEPTION_BOX_REGISTRATION_H
#define APC_OBJECT_PERCEPTION_BOX_REGISTRATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <ros/subscriber.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <apc_capture/BoxCoordinates.h>
#include <apc_capture/apc_features.h>

namespace apc_object_perception
{
    class BoxRegistration : public nodelet::Nodelet
    {
    public:
        BoxRegistration();
        virtual ~BoxRegistration();
    private:
        void onInit() override;
        void doRegistration();

        ros::NodeHandle m_nh;

        void cloudSubCallBack(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
        ros::Subscriber m_sub_pointCloud;
        //ros::Publisher m_pub_correctedPose;
        ros::Publisher m_pub_Boxmsg;
        tf::TransformBroadcaster m_tf_broadcaster;

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloudfromSensorXYZRGB;
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr m_cloudfromSensorXYZ;
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloudfromMesh;

        std::unique_ptr<tf::TransformListener> m_tf;


        std::string m_rosPackagePath;
        std::string m_imagePath;
        std::string m_imageName;
        bool m_fakeMode = false;
        apc_capture::BoxCoordinates m_boxmsg;
        apc_object_perception::ApcFeatures m_features;

    };
} // end namespace apc_object_perception



#endif
