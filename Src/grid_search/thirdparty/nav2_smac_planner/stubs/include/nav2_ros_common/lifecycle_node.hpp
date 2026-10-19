// Minimal stub of nav2_ros_common LifecycleNode for the SmacPlanner2D decoupling probe.
// (family C / X-S S-07 ad-hoc feasibility probe; NOT upstream code)
//
// types.hpp uses nav2::LifecycleNode::SharedPtr and declare_or_get_parameter() inside
// SmootherParams::get(). That method is only used by the ROS plugin layer; the stub
// returns the provided default so the header parses and the type is complete.

#ifndef NAV2_ROS_COMMON__LIFECYCLE_NODE_HPP_
#define NAV2_ROS_COMMON__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>

namespace nav2
{

class LifecycleNode
{
public:
  using SharedPtr = std::shared_ptr<LifecycleNode>;

  LifecycleNode() = default;
  virtual ~LifecycleNode() = default;

  template<typename T>
  T declare_or_get_parameter(const std::string & /*name*/, const T & default_value)
  {
    return default_value;
  }
};

}  // namespace nav2

#endif  // NAV2_ROS_COMMON__LIFECYCLE_NODE_HPP_
