// my_costmap_node.cpp
#include "nav2_costmap_2d/costmap_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyCostmapNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyCostmapNode() : LifecycleNode("my_costmap_node")
  {
    costmap_ros_ = std::make_shared<nav2_costmap_2d::CostmapROS2>(
      "my_costmap", "", shared_from_this().get(), tf_buffer_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    costmap_ros_->on_configure(this->get_node_base_interface(),
                               this->get_node_logging_interface(),
                               this->get_node_clock_interface(),
                               this->get_node_parameters_interface(),
                               this->get_node_topics_interface(),
                               this->get_node_services_interface());
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    costmap_ros_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  std::shared_ptr<nav2_costmap_2d::CostmapROS2> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};