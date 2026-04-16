#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml
import os
import threading

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        
        # Buffer và Listener để lấy tọa độ robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.waypoints = []
        # File sẽ được lưu tại thư mục bạn đang đứng (hoặc bạn có thể set đường dẫn tuyệt đối)
        self.file_path = 'saved_waypoints.yaml' 

        self.get_logger().info("Waypoint Saver Node Đã Khởi Động.")
        self.get_logger().info("--- HƯỚNG DẪN ---")
        self.get_logger().info("1. Di chuyển robot đến vị trí mong muốn.")
        self.get_logger().info("2. Nhấn phím 'ENTER' trên terminal này để lưu điểm.")
        self.get_logger().info("3. Nhấn 'Ctrl+C' để kết thúc và xuất ra file yaml.")

        # Chạy một luồng (thread) riêng để không chặn vòng lặp ROS 2
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def wait_for_input(self):
        try:
            while rclpy.ok():
                input() # Chờ người dùng nhấn Enter
                self.save_current_pose()
        except EOFError:
            pass

    def save_current_pose(self):
        try:
            # Lấy vị trí mới nhất của base_link so với map
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                now)

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

            # Format dữ liệu theo chuẩn Pose
            wp = {
                'position': {'x': float(x), 'y': float(y), 'z': float(z)},
                'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)}
            }
            self.waypoints.append(wp)
            self.get_logger().info(f"Đã lưu điểm {len(self.waypoints)}: x={x:.2f}, y={y:.2f}")

        except TransformException as ex:
            self.get_logger().warn(f'Không thể lấy tọa độ từ map -> base_link: {ex}')

    def save_to_file(self):
        if not self.waypoints:
            self.get_logger().info("Không có điểm nào được lưu. Bỏ qua việc tạo file.")
            return

        with open(self.file_path, 'w') as f:
            yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False)
        self.get_logger().info(f"Đã lưu thành công {len(self.waypoints)} điểm vào file: {os.path.abspath(self.file_path)}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nhận lệnh Ctrl+C! Đang tiến hành lưu file...")
    finally:
        node.save_to_file()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()