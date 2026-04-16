import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty

# ==========================================
# CẤU HÌNH PHÍM BẤM
# ==========================================
msg = """
Điều khiển Robot Mecanum!
---------------------------
Cụm phím di chuyển:
   q    w    e
   a    s    d

w/s : Tiến / Lùi (trục X)
a/d : Trượt ngang Trái / Phải (trục Y)
q/e : Xoay tại chỗ Trái / Phải (trục Z)

Phím bất kỳ khác: Dừng lại (Stop)

CTRL-C để thoát
"""

moveBindings = {
    'w': (1, 0, 0),   # Tiến
    's': (-1, 0, 0),  # Lùi
    'a': (0, 1, 0),   # Trượt trái
    'd': (0, -1, 0),  # Trượt phải
    'q': (0, 0, 1),   # Xoay trái (CCW)
    'e': (0, 0, -1),  # Xoay phải (CW)
}

# Vận tốc mặc định
SPEED_LINEAR = 0.5   # m/s
SPEED_ANGULAR = 1.0  # rad/s

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # Lấy tín hiệu phím mà không chờ
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class MecanumTeleop(Node):
    def __init__(self):
        super().__init__('omni_teleop_keyboard')
        
        # Publisher gửi lệnh đến controller của bạn
        self.publisher_ = self.create_publisher(
            TwistStamped, 
            '/mobile_base_controller/reference', 
            10
        )
        self.get_logger().info('Teleop Node đã khởi động. Đang đợi tín hiệu phím...')

    def publish_cmd(self, x, y, th):
        twist_stamped = TwistStamped()
        
        # Gắn stamp và frame_id
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_footprint"
        
        # Gắn vận tốc
        twist_stamped.twist.linear.x = x * SPEED_LINEAR
        twist_stamped.twist.linear.y = y * SPEED_LINEAR
        twist_stamped.twist.linear.z = 0.0
        
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = th * SPEED_ANGULAR
        
        self.publisher_.publish(twist_stamped)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    
    node = MecanumTeleop()
    
    x = 0.0
    y = 0.0
    th = 0.0

    try:
        print(msg)
        while rclpy.ok():
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]
                node.publish_cmd(x, y, th)
                
            elif key == '\x03': # CTRL+C
                break
            else:
                x = 0.0
                y = 0.0
                th = 0.0
                # Nếu không bấm gì, liên tục gửi lệnh 0 để xe không bị trôi
                node.publish_cmd(x, y, th)
                
    except Exception as e:
        print(e)

    finally:
        # Gửi lệnh dừng hẳn trước khi tắt node
        node.publish_cmd(0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()