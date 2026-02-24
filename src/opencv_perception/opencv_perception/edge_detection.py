import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node


class ObjectDetection(Node): 
    def __init__(self):
        super().__init__('object_detection_node')
        self.image_subscriber = self.create_subscription(Image, "/camera/image_raw", self.object_detection_callback, 10)
        self.bridge_object = CvBridge()
    
    def object_detection_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return 
        # Use full frame for now; adjust ROI once camera pose is fixed.
        cropped_image = cv_image
        cv2.imshow("cropped", cropped_image)
        cv2.imshow("original", cv_image)
        #convert image to gray 
        gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray", gray)
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 3, 3)
        cv2.imshow("mask", mask)
        #find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours: 
            cv2.polylines(cropped_image, [cnt], True, [255,0,0], 1)
        object_detected = []
        for cnt in contours: 
            area = cv2.contourArea(cnt)
            if area > 20: 
                cnt = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
                object_detected.append(cnt)
        print("how many objects I detect:", len(object_detected))
        print(object_detected)
        for cnt in contours: 
            rect = cv2.minAreaRect(cnt)
            (x_center, y_center), (w,h), orientation = rect
            box = cv2.boxPoints(rect)
            box = np.int0(box) 
            cv2.polylines(cropped_image,[box],True,(255,0,0))
            cv2.putText(
                cropped_image,
                f"x:{round(x_center, 1)} y:{round(y_center, 1)}",
                (int(x_center), int(y_center)),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                (0, 255, 0),
            )
            cv2.circle(cropped_image, (int(x_center), int(y_center)), 1, (255,0,0), thickness=-1)    
        cv2.imshow("detected_objects", cropped_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()

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
