import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import yaml
import math
from tf_transformations import euler_from_quaternion  # To convert quaternion to euler angles

class Create3ControlNode(Node):
    def __init__(self):
        super().__init__('create3_control_node')

        # Get target position from YAML file.
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_x', None),
                ('target_y', None)
            ])

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to /odom topic to get the robot's position and orientation
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)  # Subscribe to Odometry to track current position
        
        # Initialize current position and orientation to None
        self.current_x = None
        self.current_y = None
        self.current_orientation = None  # Orientation in radians

    def odom_callback(self, msg):
        # Update the robot's current position and orientation from the odometry data
        if self.current_x is None and self.current_y is None:
            # Initialize position with the first received odometry message
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            
            # Convert quaternion to Euler angles to get orientation (yaw)
            orientation_q = msg.pose.pose.orientation
            _, _, self.current_orientation = euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            )
            self.get_logger().info(f"Initial position: ({self.current_x}, {self.current_y}), orientation: {self.current_orientation}")
        else:
            # Continuously update position and orientation
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            
            orientation_q = msg.pose.pose.orientation
            _, _, self.current_orientation = euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            )

    def move_robot_to_target(self):
        target_x = self.target_x
        target_y = self.target_y 

        # Move the robot towards the target position (target_x, target_y)
        move_cmd = Twist()

        # Make sure the robot has received its initial position and orientation before attempting to move
        if self.current_x is None or self.current_y is None or self.current_orientation is None:
            self.get_logger().warn("Waiting for odometry data...")
            return False  # Wait for position and orientation initialization

        # Calculate the distance to the target position
        distance = math.sqrt((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2)

        # Calculate the angle to the target (from the robot's current position)
        target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
        
        # Normalize the angle to be between -pi and pi
        angle_diff = target_angle - self.current_orientation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize the angle between -pi and pi

        # Turn the robot towards the target
        if abs(angle_diff) > 0.1:
            # Rotate the robot to face the target
            move_cmd.angular.z = 0.3 if angle_diff > 0 else -0.3  # Turn at a fixed speed
            self.publisher_.publish(move_cmd)
            self.get_logger().info(f"Turning to target: angle_diff = {angle_diff}")
            return False  # Don't move until robot is facing the target

        # Once facing the target, move towards it in a straight line
        if distance > 0.1:
            move_cmd.linear.x = 0.2  # Speed in m/s
        else:
            move_cmd.linear.x = 0.0  # Stop once the robot is close to the target

        # Publish the twist message to move the robot
        self.publisher_.publish(move_cmd)
        self.get_logger().info(f"Moving to ({target_x}, {target_y}), distance: {distance}")
        return distance < 0.1  # Return True if the robot has reached the target

# def read_target_from_yaml(file_path):
#     # Read the target position from a YAML file
#     with open(file_path, 'r') as file:
#         data = yaml.safe_load(file)
#     return data['target_position']['x'], data['target_position']['y']

def main(args=None):
    rclpy.init(args=args)
    node = Create3ControlNode()

    # Move the robot to the target position
    while rclpy.ok():
        if node.move_robot_to_target():
            break  # Stop moving once the target is reached
        rclpy.spin_once(node)  # Allow the robot to update its position

    rclpy.shutdown()

if __name__ == '__main__':
    main()
