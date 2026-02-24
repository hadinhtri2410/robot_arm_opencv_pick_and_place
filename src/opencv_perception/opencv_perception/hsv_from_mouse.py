import argparse

import cv2 as cv
import numpy as np


img = None


def mouse_rgb(event, x, y, flags, param):
    del flags, param
    if event == cv.EVENT_LBUTTONDOWN:
        colors_b = img[y, x, 0]
        colors_g = img[y, x, 1]
        colors_r = img[y, x, 2]
        colors = img[y, x]
        hsv_value = np.uint8([[[colors_b, colors_g, colors_r]]])
        hsv = cv.cvtColor(hsv_value, cv.COLOR_BGR2HSV)
        print("HSV:", hsv)
        print("Red:", colors_r)
        print("Green:", colors_g)
        print("Blue:", colors_b)
        print("BGR Format:", colors)
        print("Coordinates of pixel: X:", x, "Y:", y)


def main():
    parser = argparse.ArgumentParser(description="Read HSV/BGR values from mouse clicks.")
    parser.add_argument(
        "--image",
        default="/home/triha/ros2/src/opencv_perception/images/output/camera_image.png",
        help="Path to image file",
    )
    args = parser.parse_args()

    global img
    img = cv.imread(args.image)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {args.image}")

    cv.namedWindow("mouseRGB")
    cv.setMouseCallback("mouseRGB", mouse_rgb)

    while True:
        cv.imshow("mouseRGB", img)
        if cv.waitKey(20) & 0xFF == 27:
            break

    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
