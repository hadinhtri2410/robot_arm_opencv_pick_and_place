import argparse

import cv2


def main() -> None:
    parser = argparse.ArgumentParser(description="Display a static image with OpenCV.")
    parser.add_argument(
        "--image",
        default="/home/triha/ros2/src/opencv_perception/images/input/lc563.png",
        help="Path to image file",
    )
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {args.image}")

    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
