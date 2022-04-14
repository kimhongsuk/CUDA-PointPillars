#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <pcl/console/parse.h>

#include "pointcloud_vlp16.h"


PointCloudVLP16::PointCloudVLP16()
{
    cloud_.reset( new pcl::PointCloud<PointType>() );
    test_.reset( new pcl::PointCloud<PointType>() );

    grabber_ = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress_ ), boost::lexical_cast<unsigned short>( port_ ) ) );
	viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer>( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    color_handler_ = boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>>( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
    handler_ = color_handler_;

    // PCL Visualizer
    viewer_->addCoordinateSystem( 3.0, "coordinate" );
    viewer_->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer_->initCameraParameters();
    viewer_->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    // Retrieved Point Cloud Callback Function
    function_ = [this]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( this->mutex_ );
            this->cloud_ = ptr;
        };

    // Register Callback Function
    connection_ = grabber_->registerCallback( function_ );

    // Start Grabber
    grabber_->start();

    return;
}

PointCloudVLP16::~PointCloudVLP16()
{
    // Stop Grabber
    grabber_->stop();

    // Disconnect Callback Function
    if( connection_.connected() ){
        connection_.disconnect();
    }

    return;
}

Eigen::Quaternionf PointCloudVLP16::ToQuaternion(float yaw, float pitch, float roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    float cy = std::cos(yaw * 0.5);
    float sy = std::sin(yaw * 0.5);
    float cp = std::cos(pitch * 0.5);
    float sp = std::sin(pitch * 0.5);
    float cr = std::cos(roll * 0.5);
    float sr = std::sin(roll * 0.5);

    float w = cr * cp * cy + sr * sp * sy;
    float x = sr * cp * cy - cr * sp * sy;
    float y = cr * sp * cy + sr * cp * sy;
    float z = cr * cp * sy - sr * sp * cy;

    Eigen::Quaternionf q = Eigen::Quaternionf(w, x, y, z);

    return q;
}

void PointCloudVLP16::clear() 
{
    test_.reset( new pcl::PointCloud<PointType>() );
}

void PointCloudVLP16::view(std::vector<Bndbox> &nms_pred)
{
    handler_->setInputCloud(test_);
    std::cout << "checkpoint 0" << std::endl;

    std::cout << "checkpoint 1" << std::endl;
    viewer_->removeAllShapes();
    viewer_->removeAllPointClouds();
    viewer_->updatePointCloud(test_, *handler_, "cloud");
    viewer_->addPointCloud(test_, *handler_, "cloud");
    std::cout << "checkpoint 2" << std::endl;
    for(int count = 0; count < nms_pred.size(); count++)
    {
        std::cout << "checkpoint 3" << std::endl;
        std::cout << nms_pred.size() << std::endl;
        viewer_->addCube(Eigen::Vector3f(nms_pred[count].x, nms_pred[count].y, nms_pred[count].z),
                            Eigen::Quaternionf(PointCloudVLP16::ToQuaternion(nms_pred[count].rt, 0.0, 0.0)),
                            nms_pred[count].w, nms_pred[count].h, nms_pred[count].l, std::to_string(count));
    }
}

pcl::PointCloud<PointType>::ConstPtr PointCloudVLP16::getdata()
{
    return cloud_;
}

size_t PointCloudVLP16::size()
{
    return cloud_->size();
}
