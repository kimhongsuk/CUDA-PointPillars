#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pointcloud_vlp16.h"


PointCloudVLP16::PointCloudVLP16()
{
    cloud_.reset( new pcl::PointCloud<PointType>() );

    // VLP Grabber
    grabber_ = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress_ ), boost::lexical_cast<unsigned short>( port_ ) ) );

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

pcl::PointCloud<PointType>::ConstPtr PointCloudVLP16::getdata()
{
    return cloud_;
}

size_t PointCloudVLP16::size()
{
    return cloud_->size();
}