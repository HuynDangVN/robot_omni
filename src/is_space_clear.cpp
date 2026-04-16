#include "robot_omni/is_space_clear.hpp"
#include "behaviortree_cpp/bt_factory.h" 

namespace robot_omni
{

// ==============================================================================
// IMPLEMENTATION: IsSpaceClear
// ==============================================================================
IsSpaceClear::IsSpaceClear(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("is_space_clear_bt_node");
  
  rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
  costmap_qos.transient_local().reliable();
  
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", costmap_qos,
    std::bind(&IsSpaceClear::costmapCallback, this, std::placeholders::_1));
    
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&IsSpaceClear::odomCallback, this, std::placeholders::_1));
}

BT::PortsList IsSpaceClear::providedPorts() {
  return {
    BT::InputPort<std::string>("direction"),
    BT::InputPort<double>("distance"),
    BT::InputPort<double>("radius")
  };
}

void IsSpaceClear::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { latest_costmap_ = msg; }
void IsSpaceClear::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto & q = msg->pose.pose.orientation;
  robot_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

BT::NodeStatus IsSpaceClear::tick() {
  rclcpp::spin_some(node_);
  if (!latest_costmap_) return BT::NodeStatus::FAILURE;

  std::string direction = getInput<std::string>("direction").value();
  int width = latest_costmap_->info.width;
  int height = latest_costmap_->info.height;
  double res = latest_costmap_->info.resolution;
  int cx = width / 2; int cy = height / 2;

  if (direction == "circular") {
    double radius = getInput<double>("radius").value_or(0.35);
    int cell_r = radius / res;
    for (int i = -cell_r; i <= cell_r; i++) {
      for (int j = -cell_r; j <= cell_r; j++) {
        if (i*i + j*j <= cell_r*cell_r) {
          int idx = (cy + j) * width + (cx + i);
          if (idx >= 0 && idx < width * height && latest_costmap_->data[idx] > 90) 
            return BT::NodeStatus::FAILURE; 
        }
      }
    }
  } 
  else if (direction == "backward") {
    double dist = getInput<double>("distance").value_or(0.4);
    double robot_radius = 0.25;
    // Quét vùng chữ nhật phía sau lưng xe dựa trên robot_yaw_
    for (double d = 0.0; d <= dist; d += res) {
      for (double w = -robot_radius; w <= robot_radius; w += res) {
        // Tọa độ lùi lại (yaw + PI) và dịch ngang
        int px = cx + (d * cos(robot_yaw_ + M_PI) - w * sin(robot_yaw_ + M_PI)) / res;
        int py = cy + (d * sin(robot_yaw_ + M_PI) + w * cos(robot_yaw_ + M_PI)) / res;
        if (px >= 0 && px < width && py >= 0 && py < height) {
          if (latest_costmap_->data[py * width + px] > 90) return BT::NodeStatus::FAILURE; 
        }
      }
    }
  }
  return BT::NodeStatus::SUCCESS;
}


// ==============================================================================
// IMPLEMENTATION: IsStuckInDoorway
// ==============================================================================
IsStuckInDoorway::IsStuckInDoorway(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("is_stuck_doorway_bt_node");
  rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
  costmap_qos.transient_local().reliable();
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", costmap_qos, std::bind(&IsStuckInDoorway::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&IsStuckInDoorway::odomCallback, this, std::placeholders::_1));
}

BT::PortsList IsStuckInDoorway::providedPorts() { return {}; }
void IsStuckInDoorway::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { latest_costmap_ = msg; }
void IsStuckInDoorway::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto & q = msg->pose.pose.orientation;
  robot_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

BT::NodeStatus IsStuckInDoorway::tick() {
  rclcpp::spin_some(node_);
  if (!latest_costmap_) return BT::NodeStatus::FAILURE;

  int width = latest_costmap_->info.width;
  int height = latest_costmap_->info.height;
  double res = latest_costmap_->info.resolution;
  int cx = width / 2; int cy = height / 2;

  bool left_blocked = false;
  bool right_blocked = false;

  // Quét tia sang trái (yaw + 90 độ)
  for (double d = 0.25; d <= 0.6; d += res) {
    int px = cx + (d * cos(robot_yaw_ + M_PI/2.0)) / res;
    int py = cy + (d * sin(robot_yaw_ + M_PI/2.0)) / res;
    if (px >= 0 && px < width && py >= 0 && py < height && latest_costmap_->data[py * width + px] > 90) { left_blocked = true; break; }
  }
  // Quét tia sang phải (yaw - 90 độ)
  for (double d = 0.25; d <= 0.6; d += res) {
    int px = cx + (d * cos(robot_yaw_ - M_PI/2.0)) / res;
    int py = cy + (d * sin(robot_yaw_ - M_PI/2.0)) / res;
    if (px >= 0 && px < width && py >= 0 && py < height && latest_costmap_->data[py * width + px] > 90) { right_blocked = true; break; }
  }

  // Nếu kẹp cả 2 bên -> Đang ở trong cửa
  if (left_blocked && right_blocked) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

// ==============================================================================
// IMPLEMENTATION: IsPathForward
// ==============================================================================
IsPathForward::IsPathForward(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf) {}

BT::PortsList IsPathForward::providedPorts() {
  return { BT::InputPort<nav_msgs::msg::Path>("path", "The global path") };
}

BT::NodeStatus IsPathForward::tick() {
  auto path_exp = getInput<nav_msgs::msg::Path>("path");
  if (!path_exp) return BT::NodeStatus::FAILURE;
  auto path = path_exp.value();
  
  if (path.poses.size() < 5) return BT::NodeStatus::SUCCESS; // Path quá ngắn, mặc định tiến

  // Lấy góc quay xuất phát của path (chính là đầu xe)
  auto & q = path.poses[0].pose.orientation;
  double start_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Lấy điểm đích ở đoạn ngắn phía trước để tính vector hướng đi
  int lookahead_idx = std::min(15, (int)path.poses.size() - 1);
  double dx = path.poses[lookahead_idx].pose.position.x - path.poses[0].pose.position.x;
  double dy = path.poses[lookahead_idx].pose.position.y - path.poses[0].pose.position.y;
  
  double path_angle = atan2(dy, dx);
  // Tính độ lệch góc giữa hướng xe và hướng path (tuyệt đối)
  double diff = std::abs(atan2(sin(path_angle - start_yaw), cos(path_angle - start_yaw)));

  // Nếu lệch < 90 độ (PI/2) -> Path nằm ở nửa trước mặt
  if (diff < M_PI / 2.0) return BT::NodeStatus::SUCCESS; 
  return BT::NodeStatus::FAILURE; // Nằm ở nửa sau lưng
}

// ==============================================================================
// IMPLEMENTATION: CalculateEscapeAngle (TÍNH GÓC XOAY TỐI ƯU KHI GẶP VẬT CẢN)
// ==============================================================================
CalculateEscapeAngle::CalculateEscapeAngle(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  node_ = rclcpp::Node::make_shared("calc_escape_angle_node");
  rclcpp::QoS costmap_qos(rclcpp::KeepLast(1));
  costmap_qos.transient_local().reliable();
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", costmap_qos, std::bind(&CalculateEscapeAngle::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&CalculateEscapeAngle::odomCallback, this, std::placeholders::_1));
}

BT::PortsList CalculateEscapeAngle::providedPorts() {
  return { BT::OutputPort<double>("escape_angle", "Góc xoay tối ưu (radian)") };
}

void CalculateEscapeAngle::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { latest_costmap_ = msg; }
void CalculateEscapeAngle::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto & q = msg->pose.pose.orientation;
  robot_yaw_ = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

BT::NodeStatus CalculateEscapeAngle::tick() {
  rclcpp::spin_some(node_);
  if (!latest_costmap_) {
    setOutput("escape_angle", 0.35); // Nếu không có map, mặc định quay trái 20 độ
    return BT::NodeStatus::SUCCESS;
  }

  int width = latest_costmap_->info.width;
  int height = latest_costmap_->info.height;
  double res = latest_costmap_->info.resolution;
  int cx = width / 2; int cy = height / 2;

  double sum_x = 0.0, sum_y = 0.0, total_cost = 0.0;
  int cell_r = 0.6 / res; // Quét bán kính 0.6m quanh robot

  for (int i = -cell_r; i <= cell_r; i++) {
    for (int j = -cell_r; j <= cell_r; j++) {
      if (i*i + j*j <= cell_r*cell_r) {
        int px = cx + i; int py = cy + j;
        if (px >= 0 && px < width && py >= 0 && py < height) {
          int cost = latest_costmap_->data[py * width + px];
          if (cost > 50) { // Chỉ quan tâm các cell có cost cao (vật cản thật sự)
            double cell_angle = atan2(j * res, i * res);
            double rel_angle = cell_angle - robot_yaw_;
            // Chuẩn hóa góc về khoảng [-PI, PI]
            while (rel_angle > M_PI) rel_angle -= 2.0 * M_PI;
            while (rel_angle < -M_PI) rel_angle += 2.0 * M_PI;

            // Chỉ lấy vùng 180 độ phía trước mặt mũi xe
            if (rel_angle > -M_PI/2.0 && rel_angle < M_PI/2.0) {
              sum_x += cos(rel_angle) * cost;
              sum_y += sin(rel_angle) * cost;
              total_cost += cost;
            }
          }
        }
      }
    }
  }

  double escape_angle = 0.35; // Mặc định 20 độ (0.35 rad)
  if (total_cost > 0) {
    // Tính góc trọng tâm của khối chướng ngại vật
    double obstacle_angle = atan2(sum_y, sum_x); 
    
    // Xoay hướng ngược lại + margin 20 độ (0.35 rad) an toàn
    if (obstacle_angle > 0) { // Vật cản nằm bên TRÁI
      escape_angle = -(obstacle_angle + 0.35); // Phải xoay PHẢI (âm)
    } else {                  // Vật cản nằm bên PHẢI
      escape_angle = -(obstacle_angle - 0.35); // Phải xoay TRÁI (dương)
    }
  }

  // Khóa góc xoay tối đa là 90 độ (1.57 rad) để robot không xoay vòng vòng mất phương hướng
  if (escape_angle > 1.57) escape_angle = 1.57;
  if (escape_angle < -1.57) escape_angle = -1.57;

  RCLCPP_INFO(node_->get_logger(), "Tinh duoc goc ne vat can: %.2f rad", escape_angle);
  setOutput("escape_angle", escape_angle);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robot_omni

// =====================================================================
// ĐĂNG KÝ TẤT CẢ PLUGIN VÀO BEHAVIOR TREE FACTORY
// =====================================================================
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robot_omni::IsSpaceClear>("IsSpaceClear");
  factory.registerNodeType<robot_omni::IsStuckInDoorway>("Custom_IsStuckInDoorway");
  factory.registerNodeType<robot_omni::IsPathForward>("Custom_IsPathForward");
  factory.registerNodeType<robot_omni::CalculateEscapeAngle>("Custom_CalculateEscapeAngle"); 
}