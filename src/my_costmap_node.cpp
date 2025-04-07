// my_costmap_node.cpp
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}