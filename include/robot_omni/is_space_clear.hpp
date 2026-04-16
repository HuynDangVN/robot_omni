#ifndef IS_SPACE_CLEAR_HPP_
#define IS_SPACE_CLEAR_HPP_

#include <string>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h" 
#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot_omni
{

// =========================================================
// NODE 1: Kiểm tra không gian an toàn (Lùi / Xoay)
// =========================================================
class IsSpaceClear : public BT::ConditionNode
{
public:
  IsSpaceClear(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  double robot_yaw_ = 0.0;
};

// =========================================================
// NODE 2: Kiểm tra kẹt ở cửa (Tường kẹp 2 bên)
// =========================================================
class IsStuckInDoorway : public BT::ConditionNode
{
public:
  IsStuckInDoorway(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  double robot_yaw_ = 0.0;
};

// =========================================================
// NODE 3: Kiểm tra hướng của quỹ đạo (Tiến hay Lùi)
// =========================================================
class IsPathForward : public BT::ConditionNode
{
public:
  IsPathForward(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

// =========================================================
// NODE 4: Tính toán góc xoay né vật cản tối ưu
// =========================================================
class CalculateEscapeAngle : public BT::SyncActionNode
{
public:
  CalculateEscapeAngle(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  double robot_yaw_ = 0.0;
};

}  // namespace robot_omni

#endif  // IS_SPACE_CLEAR_HPP_