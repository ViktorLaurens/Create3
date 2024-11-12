import rclpy
from rclpy.logging import get_logger

def main():
    # Initialize the ROS 2 Python client library
    rclpy.init()

    # Create a Node
    node = rclpy.create_node('test_node')

    logger = get_logger('test_logger')
    logger.info('ROS 2 Python modules are working!')  # ROS 2 logging instead of print

    # Spin the node (this would normally be used to keep the node alive for processing)
    rclpy.spin(node)

    # Shutdown ROS 2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
