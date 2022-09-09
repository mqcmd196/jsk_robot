#include <complex_recovery/concurrent_complex_recovery.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(complex_recovery::ConcurrentComplexRecovery, nav_core::RecoveryBehavior)

namespace complex_recovery
{

ConcurrentComplexRecovery::ConcurrentComplexRecovery(): initialized_(false)
{
}

void ConcurrentComplexRecovery::initialize(
            std::string name,
            tf2_ros::Buffer*,
            costmap_2d::Costmap2DROS* global_costmap,
            costmap_2d::Costmap2DROS* local_costmap)
{
    if (not initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        // DO SOMETHING
        initialized_ = true;
    } else {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

ConcurrentComplexRecovery::~ConcurrentComplexRecovery()
{
}

void ConcurrentComplexRecovery::runBehavior()
{
  if (not initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  // DO SOMETHING
}

};