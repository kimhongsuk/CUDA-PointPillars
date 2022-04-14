#ifndef POINTCLOUD_VLP16_H_
#define POINTCLOUD_VLP16_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/impl/eigen.hpp>

#include "postprocess.h"

// Point Type
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZI PointType;

class PointCloudVLP16 {
    public:
        PointCloudVLP16();
        ~PointCloudVLP16();

        pcl::PointCloud<PointType>::ConstPtr getdata();
        Eigen::Quaternionf ToQuaternion(float yaw, float pitch, float roll);
        void view(std::vector<Bndbox> &nms_pred);
        void clear();

        size_t size();

        pcl::PointCloud<PointType>::ConstPtr cloud_;
        pcl::PointCloud<PointType>::Ptr test_;
        boost::mutex mutex_;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    private:
        std::string ipaddress_ = "192.168.1.24";
        std::string port_ = "2368";

        boost::shared_ptr<pcl::VLPGrabber> grabber_;
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler_;
        pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler_;
        boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function_;
        boost::signals2::connection connection_;
};

#endif
