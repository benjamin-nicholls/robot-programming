import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# create publisher --> publish geometry_msgs/PoseStamped msgs
#   at position of CLOSEST laser scan reading
#   display this pose in Rviz



def main(args=None):
    rclpy.init(args=args)


if __name__ == '__main__':
    main()
