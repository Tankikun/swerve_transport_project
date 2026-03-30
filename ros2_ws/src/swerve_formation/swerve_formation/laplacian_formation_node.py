import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math

class LaplacianFormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        
        # --- PARAMETERS ---
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('neighbors', ['tb3_1']) # Who does this robot listen to?
        self.declare_parameter('k_gain', 1.5)          # Consensus aggressiveness
        
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.neighbors = self.get_parameter('neighbors').get_parameter_value().string_array_value
        self.k_gain = self.get_parameter('k_gain').get_parameter_value().double_value

        # --- VIRTUAL STRUCTURE DEFINITION ---
        # Define where each robot should be relative to the "Virtual Center" (0,0)
        # Example: tb3_0 is on the Left (y=0.5), tb3_1 is on the Right (y=-0.5)
        self.desired_offsets = {
            'tb3_0': np.array([0.0, 0.5]),
            'tb3_1': np.array([0.0, -0.5])
        }

        # State storage
        self.my_pose = np.array([0.0, 0.0])
        self.neighbor_poses = {neighbor: np.array([0.0, 0.0]) for neighbor in self.neighbors}

        # --- PUBLISHERS & SUBSCRIBERS ---
        # 1. Subscribe to MY odometry
        self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_callback, 10)
        
        # 2. Subscribe to NEIGHBORS' odometry (This leverages Zenoh's decentralized pub/sub)
        for neighbor in self.neighbors:
            self.create_subscription(
                Odometry, 
                f'/{neighbor}/odom', 
                lambda msg, n=neighbor: self.neighbor_callback(msg, n), 
                10
            )

        # 3. Subscribe to the Virtual Center's commanded velocity (Teleop)
        self.virtual_vel = np.array([0.0, 0.0])
        self.create_subscription(Twist, '/virtual_center/cmd_vel', self.virtual_cmd_callback, 10)

        # 4. Publisher for MY calculated velocity (Sent to OpenCR Swerve IK)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)

        # Control Loop Timer (Runs at 20 Hz)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Laplacian Controller started for {self.robot_id}")

    def odom_callback(self, msg):
        self.my_pose[0] = msg.pose.pose.position.x
        self.my_pose[1] = msg.pose.pose.position.y

    def neighbor_callback(self, msg, neighbor_id):
        self.neighbor_poses[neighbor_id][0] = msg.pose.pose.position.x
        self.neighbor_poses[neighbor_id][1] = msg.pose.pose.position.y

    def virtual_cmd_callback(self, msg):
        # The base velocity the whole formation should move at
        self.virtual_vel[0] = msg.linear.x
        self.virtual_vel[1] = msg.linear.y

    def control_loop(self):
        # --- THE GRAPH LAPLACIAN MATH ---
        
        consensus_velocity = np.array([0.0, 0.0])
        my_desired = self.desired_offsets[self.robot_id]

        # Calculate the Laplacian sum: Sum of (State Difference - Desired Difference)
        for neighbor in self.neighbors:
            neighbor_desired = self.desired_offsets[neighbor]
            
            # Actual distance vector between me and neighbor
            actual_diff = self.my_pose - self.neighbor_poses[neighbor]
            
            # Where we SHOULD be relative to each other
            desired_diff = my_desired - neighbor_desired
            
            # The Error
            error = actual_diff - desired_diff
            
            # Accumulate the consensus velocity correction
            consensus_velocity -= self.k_gain * error

        # --- FINAL VELOCITY ---
        # Final = Feedforward (Virtual Center Speed) + Feedback (Laplacian Correction)
        final_vx = self.virtual_vel[0] + consensus_velocity[0]
        final_vy = self.virtual_vel[1] + consensus_velocity[1]

        # Publish to the low-level Swerve IK node
        cmd = Twist()
        cmd.linear.x = final_vx
        cmd.linear.y = final_vy
        # Note: Angular Z (rotation of the virtual structure) requires a rotational
        # transformation matrix added to the Laplacian, kept simple here for planar X/Y.
        cmd.angular.z = 0.0 
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LaplacianFormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()