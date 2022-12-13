#include "pcl_upsample/pcl_upsample_nodelet.hpp"

namespace pcl_upsample
{
    void PclUpsampleNodelet::onInit()
    {
        pcl_upsample_ptr_.reset(new PclUpsample(getPrivateNodeHandle()));
        if (!pcl_upsample_ptr_->initialize())
        {
            ROS_ERROR("Cannot initialize PCL upsampling node.");
            return;
        }
        return;
    }

    PLUGINLIB_EXPORT_CLASS(pcl_upsample::PclUpsampleNodelet,
                           nodelet::Nodelet);
} // namespace pcl_upsample