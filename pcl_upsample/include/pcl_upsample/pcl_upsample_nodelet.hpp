#ifndef PCL_UPSAMPLE_NODELET_HPP_
#define PCL_UPSAMPLE_NODELET_HPP_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "pcl_upsample/pcl_upsample.hpp"

namespace pcl_upsample
{
    class PclUpsampleNodelet : public nodelet::Nodelet
    {
    public:
        PclUpsampleNodelet()
        {
            return;
        }
        ~PclUpsampleNodelet()
        {
            return;
        }

    private:
        virtual void onInit();
        PclUpsample::Ptr pcl_upsample_ptr_;
    };
} // namespace pcl_upsample

#endif