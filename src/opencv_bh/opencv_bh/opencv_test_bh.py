import rclpy
from rclpy.node import Node

from cv2 import cvtColor, imshow, COLOR_BGR2GRAY

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageConverter(Node):

    def __init__(self):
        super().__init__('opencv_test_bh')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 
                                                "/limo/depth_camera_link/image_raw",
                                                self.image_callback,
                                                10)
    
def image_callback(self, data):
    pass


def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()