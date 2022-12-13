#ifndef PCL_UPSAMPLE_NODE_HPP_
#define PCL_UPSAMPLE_NODE_HPP_

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_upsample
{
    class PclUpsample
    {
    public:
        PclUpsample(ros::NodeHandle &nh);
        ~PclUpsample();

        bool initialize();

        typedef boost::shared_ptr<PclUpsample> Ptr;
        typedef boost::shared_ptr<const PclUpsample> ConstPtr;

    private:
        bool loadParams();
        bool createRosIo();
        void downPclCb(const sensor_msgs::PointCloud2ConstPtr &input_msg);

        ros::NodeHandle nh_;
        ros::Subscriber down_pcl_sub_;
        ros::Publisher up_pcl_pub_;

        double pcl_search_radius_, pcl_upsample_size_;
    };
} // pcl_upsample

#endif