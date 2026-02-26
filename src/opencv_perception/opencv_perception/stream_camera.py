import cv2
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from cv_bridge import CvBridge, CvBridgeError
from my_robot_interfaces.srv import GetPosition
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection_node")

        self.bridge_object = CvBridge()

        self.image_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.object_detection_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_callback, 10
        )
        self.camera_server = self.create_service(
            GetPosition, "/get_position", self.stream_camera_callback
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_cam_info = None
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.camera_frame = self.declare_parameter("camera_frame", "camera_optical_link").value
        # In this UR3e Gazebo setup, table top is z=0.0 in base_link coordinates.
        self.z_table = float(self.declare_parameter("table_z_in_base", 0.0).value)
        self.last_tf_warn_time_sec = -1.0
        self.get_logger().info(
            f"Perception frames: base='{self.base_frame}', camera='{self.camera_frame}', table_z_in_base={self.z_table:.3f}"
        )

        # Each color maps to per-type detections:
        # {"block": {...}, "bin": {...}, "any": {...}}
        self.detected_objects = {}

    def camera_info_callback(self, msg: CameraInfo):
        self.last_cam_info = msg

    def pixel_to_base_xy_on_table(self, u, v):
        if self.last_cam_info is None:
            return None

        try:
            tf_base_from_cam = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
            )
        except Exception as exc:  # noqa: BLE001
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if self.last_tf_warn_time_sec < 0.0 or (now_sec - self.last_tf_warn_time_sec) > 2.0:
                self.get_logger().warn(f"TF lookup failed: {exc}")
                self.last_tf_warn_time_sec = now_sec
            return None

        cam_info = self.last_cam_info
        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx = cam_info.k[2]
        cy = cam_info.k[5]

        if fx == 0.0 or fy == 0.0:
            return None

        r_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)

        tx = tf_base_from_cam.transform.translation.x
        ty = tf_base_from_cam.transform.translation.y
        tz = tf_base_from_cam.transform.translation.z
        camera_origin = np.array([tx, ty, tz], dtype=float)

        q = tf_base_from_cam.transform.rotation
        rotation = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        ray_base = rotation @ r_cam

        if abs(ray_base[2]) < 1e-9:
            return None

        scale = (self.z_table - camera_origin[2]) / ray_base[2]
        if scale <= 0.0:
            return None

        return camera_origin + scale * ray_base

    def _collect_contour_candidates(self, mask, min_area=80.0):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < min_area:
                continue
            (_, _), (width, height), _ = cv2.minAreaRect(cnt)
            if width < 6.0 or height < 6.0:
                continue
            candidates.append((area, cnt))
        return candidates

    def _select_contour(self, candidates, target_type):
        if not candidates:
            return None

        # Bins and blocks share color; bins are much larger.
        if target_type == "bin":
            return max(candidates, key=lambda item: item[0])[1]

        # "block" and "any" default to the smallest significant contour.
        return min(candidates, key=lambda item: item[0])[1]

    def _compute_grasp_joint_goal(self, x, y):
        # Heuristic grasp joint target used by pick_and_place.
        depth = max(0.45, abs(y) + 0.45)
        pan = float(np.clip(np.arctan2(x, depth), -0.8, 0.8))
        goal = [-0.31, -1.51, -1.00, 0.71, 0.0, 0.0]
        goal[0] = pan
        return goal

    def object_detection_callback(self, data: Image):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(str(exc))
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        lower_blue = np.array([100, 120, 50])
        upper_blue = np.array([130, 255, 255])
        lower_yellow = np.array([20, 120, 80])
        upper_yellow = np.array([35, 255, 255])

        masks = {
            "red": cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2),
            "blue": cv2.inRange(hsv, lower_blue, upper_blue),
            "yellow": cv2.inRange(hsv, lower_yellow, upper_yellow),
        }
        draw_colors = {
            "red": (0, 0, 255),
            "blue": (255, 0, 0),
            "yellow": (0, 255, 255),
        }

        new_detections = {}

        for color_name, mask in masks.items():
            candidates = self._collect_contour_candidates(mask)
            if not candidates:
                continue

            per_type = {}
            for target_type in ("block", "bin"):
                contour = self._select_contour(candidates, target_type)
                if contour is None:
                    continue

                area = float(cv2.contourArea(contour))
                rect = cv2.minAreaRect(contour)
                (x_center, y_center), _, _ = rect
                box = cv2.boxPoints(rect).astype(np.int32)

                world_point = self.pixel_to_base_xy_on_table(float(x_center), float(y_center))
                if world_point is None:
                    continue

                per_type[target_type] = {
                    "type": target_type,
                    "area": area,
                    "x": float(world_point[0]),
                    "y": float(world_point[1]),
                    "z": float(world_point[2]),
                    "u": float(x_center),
                    "v": float(y_center),
                    "goal_joint_positions": self._compute_grasp_joint_goal(
                        float(world_point[0]), float(world_point[1])
                    ),
                }

                # Keep visualization uncluttered: draw the block candidate.
                if target_type == "block":
                    cv2.polylines(cv_image, [box], True, draw_colors[color_name], 2)
                    cv2.circle(cv_image, (int(x_center), int(y_center)), 3, draw_colors[color_name], thickness=-1)
                    cv2.putText(
                        cv_image,
                        f"{color_name}-{target_type}: x:{world_point[0]:.3f} y:{world_point[1]:.3f} a:{area:.0f}",
                        (int(x_center) + 5, int(y_center) - 5),
                        cv2.FONT_HERSHEY_PLAIN,
                        1,
                        (0, 255, 0),
                        1,
                    )

            if not per_type:
                continue

            per_type["any"] = per_type.get("block", per_type.get("bin"))
            new_detections[color_name] = per_type

        self.detected_objects = new_detections
        cv2.imshow("stream_camera", cv_image)
        cv2.waitKey(1)

    def stream_camera_callback(self, request, response):
        response.found = False
        response.color = ""
        response.type = ""
        response.x_position = float("nan")
        response.y_position = float("nan")
        response.z_position = float("nan")
        response.goal_joint_positions = []

        if not request.get_position:
            return response

        requested = request.target_color.strip().lower()
        target_type = request.target_type.strip().lower() if request.target_type else "any"
        if target_type not in ("any", "block", "bin"):
            target_type = "any"

        if requested in self.detected_objects:
            per_type = self.detected_objects[requested]
            obj = per_type.get(target_type, per_type.get("any"))
            if obj is None:
                return response
            response.found = True
            response.color = requested
            response.type = obj["type"]
            response.x_position = obj["x"]
            response.y_position = obj["y"]
            response.z_position = obj["z"]
            response.goal_joint_positions = obj["goal_joint_positions"]
            return response

        if requested in ("", "any") and self.detected_objects:
            typed_candidates = []
            for color_name, per_type in self.detected_objects.items():
                obj = per_type.get(target_type, per_type.get("any"))
                if obj is not None:
                    typed_candidates.append((obj["area"], color_name, obj))

            if not typed_candidates:
                return response

            _, best_color, obj = max(typed_candidates, key=lambda item: item[0])
            response.found = True
            response.color = best_color
            response.type = obj["type"]
            response.x_position = obj["x"]
            response.y_position = obj["y"]
            response.z_position = obj["z"]
            response.goal_joint_positions = obj["goal_joint_positions"]
            return response

        self.get_logger().warn(
            f"Requested color '{requested}' not found. Current colors: {list(self.detected_objects.keys())}"
        )
        return response


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
