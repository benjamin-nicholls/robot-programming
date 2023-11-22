import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
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
        # Convert input image to cv2 format.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')  # 'bgr8'
        except CvBridgeError as e:
            print(e)
        
        # Convert BGR to HSV.
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        cv2.namedWindow('Orignal Image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Orignal Image', cv_image)

        # Get mask for orange.
        # H [0,179], S [0,255], V [0,255].
        # 10-16, 75-95, 50-90
        L = [10,75,50]
        U =  [16,95,90]
        lower_thresh = (179/100*L[0], 255/100*L[1], 255/100*L[2])
        upper_thresh = (179/100*U[0], 255/100*U[1], 255/100*U[2])
        masked_image = cv2.inRange(cv_image, lower_thresh, upper_thresh)
        cv2.namedWindow('Mask', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Mask', masked_image)

        new_image = numpy.bitwise_and(cv_image, masked_image)
        cv2.namedWindow('new window', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('new window', new_image)
        
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()