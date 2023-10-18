import rclpy
from rclpy.node import Node

from cv2 import namedWindow, inRange, imshow, waitKey
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy

class ImageConverter(Node):

    def __init__(self):
        super().__init__('opencv_test_bh')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 
                                                "/limo/depth_camera_link/image_raw",
                                                self.image_callback,
                                                10)
    
    def image_callback(self, data):
        '''
        masks out any specific color in the image
        '''
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #print(cv_image)
        namedWindow('image window', 1)  # 1 is a flag to set to automatically resize
        imshow('image window', cv_image)
        ## THIS DOESN'T WORK THE WAY I THINK IT DOES
        masked_image = inRange(cv_image, (0,0,0), (160,160,160))
        print(numpy.array(masked_image))
        namedWindow('masked window', 1)
        imshow('masked window', masked_image)
        # WRONG
        new_image = numpy.logical_and(cv_image, masked_image)
        namedWindow('new window', 1)
        imshow('new window', new_image)
        
        waitKey(1)
        


def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()