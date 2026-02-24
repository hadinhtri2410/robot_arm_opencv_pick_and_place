import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ColorFilter(Node): 
    def __init__(self):
        super().__init__("color_filter_node")
        self.color_subscriber = self.create_subscription(Image, "/camera/image_raw", self.color_callback, 10)
        self.bridge_object = CvBridge()
    def color_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return 
        
        #convert image from rgb to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        res_red =   cv2.bitwise_and(cv_image, cv_image, mask = mask_red)    
        cv2.imshow('Original', cv_image)
        cv2.imshow('Mask on Color', mask_red)
        cv2.imshow("Red", res_red)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
