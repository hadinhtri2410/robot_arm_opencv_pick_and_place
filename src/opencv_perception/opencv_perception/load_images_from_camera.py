import rclpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node


class LoadImage(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge_object = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.camera_callback,
            10
        )

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return

        path = "/home/triha/ros2/src/opencv_perception/images/output/"
        cv2.imwrite(path + "camera_image.png", cv_image)
        cv2.imshow("frame from camera", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LoadImage()

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
