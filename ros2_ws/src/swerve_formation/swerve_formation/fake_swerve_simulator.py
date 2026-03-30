import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
from visualization_msgs.msg import Marker

class FakeSwerveRobot(Node):
    """
    Simulates a holonomic (Swerve/Conveyor) robot.
    Listens to /cmd_vel and integrates velocity to publish /odom.
    """
    def __init__(self):
        super().__init__('fake_swerve_robot')
        
        # Parameters for robot ID and starting position
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.x = self.get_parameter('start_x').get_parameter_value().double_value
        self.y = self.get_parameter('start_y').get_parameter_value().double_value
        
        self.vx = 0.0
        self.vy = 0.0
        
        # Subscribe to the velocities outputted by the Laplacian Controller
        self.create_subscription(Twist, f'/{self.robot_id}/cmd_vel', self.cmd_callback, 10)
        
        # Publish the simulated position
        self.odom_pub = self.create_publisher(Odometry, f'/{self.robot_id}/odom', 10)
        
        # Physics Loop (50 Hz)
        self.last_time = time.time()
        self.create_timer(0.02, self.update_physics)
        self.get_logger().info(f"Simulated Swerve Robot [{self.robot_id}] Spawned at ({self.x}, {self.y})")
        self.marker_pub = self.create_publisher(Marker, f'/{self.robot_id}/marker', 10)

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y

    def update_physics(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # Integrate velocity to get position (Basic Kinematics)
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # Construct and publish Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.robot_id
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # Keeping rotation fixed at 0 for planar translation testing
        odom.pose.pose.orientation.w = 1.0 
        
        self.odom_pub.publish(odom)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.ns = self.robot_id
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position it exactly where the robot is
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.05 # Half the height so it sits on the grid
        marker.pose.orientation.w = 1.0

        # Set the size (e.g., 20cm wide, 10cm tall)
        marker.scale.x = 0.2 
        marker.scale.y = 0.2
        marker.scale.z = 0.1

        # Set the color (Let's make tb3_0 Blue and tb3_1 Red)
        marker.color.a = 1.0 # Alpha (transparency)
        if self.robot_id == 'tb3_0':
            marker.color.b = 1.0 # Blue
        else:
            marker.color.r = 1.0 # Red

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FakeSwerveRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()