#include "pcl_upsample/pcl_upsample.hpp"

namespace pcl_upsample
{
    PclUpsample::PclUpsample(ros::NodeHandle &nh) : nh_(nh)
    {
        return;
    }

    PclUpsample::~PclUpsample()
    {
        return;
    }

    bool PclUpsample::initialize()
    {
        if (!loadParams())
        {
            return false;
        }
        ROS_INFO("Loaded ROS parameters.");
        if (!createRosIo())
        {
            return false;
        }
        ROS_INFO("Created ROS IO.");

        return true;
    }

    bool PclUpsample::loadParams()
    {
        nh_.param<double>("search_radius", pcl_search_radius_, 0.03);
        nh_.param<double>("upsample_size", pcl_upsample_size_, 0.01);

        return true;
    }

    bool PclUpsample::createRosIo()
    {
        down_pcl_sub_ = nh_.subscribe("pcl_input", 1, &PclUpsample::downPclCb, this);
        up_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);

        return true;
    }

    void PclUpsample::downPclCb(const sensor_msgs::PointCloud2ConstPtr &input_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> input_pcl;
        pcl::fromROSMsg(*input_msg, input_pcl);

        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
        filter.setInputCloud(input_pcl.makeShared());

        // Search close points.
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree;
        filter.setSearchMethod(kd_tree);
        filter.setSearchRadius(pcl_search_radius_);

        // Upsampling.
        filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
        filter.setUpsamplingRadius(pcl_search_radius_);
        filter.setUpsamplingStepSize(pcl_upsample_size_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        filter.process(*filtered_pcl);

        // Convert PCL data into ROS PointCloud2 message.
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_pcl.get(), output_msg);
        output_msg.header = input_msg->header;

        up_pcl_pub_.publish(output_msg);

        return;
    }
} // namespace pcl_upsample