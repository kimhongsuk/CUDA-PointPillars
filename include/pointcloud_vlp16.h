#ifndef POINTCLOUD_VLP16_H_
#define POINTCLOUD_VLP16_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>

// Point Type
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZI PointType;

class PointCloudVLP16 {
    public:
        PointCloudVLP16();
        ~PointCloudVLP16();

        pcl::PointCloud<PointType>::ConstPtr getdata();
        size_t size();

        pcl::PointCloud<PointType>::ConstPtr cloud_;
        boost::mutex mutex_;

    private:
        std::string ipaddress_ = "192.168.1.24";
        std::string port_ = "2368";

        // Point Cloud


        boost::shared_ptr<pcl::VLPGrabber> grabber_;
        boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function_;
        boost::signals2::connection connection_;
};

#endif
