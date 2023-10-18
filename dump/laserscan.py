import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# create publisher --> publish geometry_msgs/PoseStamped msgs
#   at position of CLOSEST laser scan reading
#   display this pose in Rviz


class ScanPublisher(Node):
    def __init__(self):
        #self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
        pass



class Scanner(Node):
    
    def __init__(self):
        # Callback called any time a new laser scan becomes available.
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)
    

    def laserscan_callback(self, data):
        ''' Find the closest laser scan reading. Return position. '''
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        min_range = data.range_min
        index = data.ranges.index(min_range)
        angle = data.angle_min + data.angle_increment * index

        '''
        range_min
        find range_min within ranges[]
        save index
        angle = angle_min + angle_increment * index
        use angle of robot + angle --> angle of closest point
        use this angle & range to calculate position (pose) of this reading
        convert into geometry_msgs/PoseStamped
        '''
        # listen to base link for localisation (not accurate)



def main(args=None):
    rclpy.init(args=args)
    # do something here

    rclpy.shutdown()


if __name__ == '__main__':
    main()
