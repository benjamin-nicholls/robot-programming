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
        self.alreadyPrinted = False
        self.verbose = True
        self.showIntermediateImages = False
    
    def image_callback(self, data):
        '''
        masks out any specific color in the image
        '''
        # Convert input image to cv2 format.
        try:
            cv_image_bgr = self.bridge.imgmsg_to_cv2(data, 'bgr8')  # 'bgr8'
        except CvBridgeError as e:
            print(e)

        if self.showIntermediateImages:
            cv2.namedWindow('1 Orignal Image', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('1 Orignal Image', cv_image_bgr)

        # Convert BGR to HSV.
        cv_image_hsv = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2HSV)

        
        # General HSV ranges: 0-360, 0-100%, 0-100%
        # OpenCV HSV ranges: H [0,179], S [0,255], V [0,255].
        
        # HSV upper and lower bounds.
        L = [10,75,50]
        U = [20,100,100]
        # Normalisation.
        lower_thresh = (L[0]/360*179, L[1]/100*255, L[2]/100*255)
        upper_thresh = (U[0]/360*179, U[1]/100*255, U[2]/100*255)

        if self.verbose and not self.alreadyPrinted:
            print(f'lower_thresh={lower_thresh}, upper_thresh={upper_thresh}')
            self.alreadyPrinted = True

        # Get mask for colour orange.
        masked_image = cv2.inRange(cv_image_hsv, lower_thresh, upper_thresh)
        if self.showIntermediateImages:
            cv2.namedWindow('2 Mask', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('2 Mask', masked_image)

        # Apply mask and get result image.
        result_image_hsv = cv2.bitwise_and(cv_image_hsv, cv_image_hsv, mask=masked_image)
        # Convert back to BGR colourspace.
        result_image_bgr = cv2.cvtColor(result_image_hsv, cv2.COLOR_HSV2BGR)
        cv2.namedWindow('Resultant Image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Resultant Image', result_image_bgr)
        
        image_canny = cv2.Canny(cv_image_bgr, 5, 300)
        cv2.namedWindow('Canny', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Canny', image_canny)


        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)

    image_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()