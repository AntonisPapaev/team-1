import cv2
import os
import re
import numpy as np
from color_hsv import hsv_ranges


def find_latest_image():
    folder = "images"
    pattern = re.compile(r"image(\d+)\.jpg")

    max_num = -1
    max_file = None

    for filename in os.listdir(folder):
        match = pattern.fullmatch(filename)
        if match:
            num = int(match.group(1))
            if num > max_num:
                max_num = num
                max_file = filename
    return f"images/{max_file}"


def show_image(img):
    cv2.imshow("img", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def calculate_blur_score(img):
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_score = cv2.Laplacian(grey, cv2.CV_64F)
    return blur_score.var()


def find_white_line(img):
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 40, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)  # within the range = white, everything else = black
    edges = cv2.Canny(mask, 50, 150)  # makes edge pixels white
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
    # makes an array of line segments - shape (N, 1, 4)
    out = img.copy()

    if lines is not None:
        for (x1, y1, x2, y2) in lines[:, 0]:  # (N, 1, 4) -> (N, 4)
            cv2.line(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
    return out


def calculate_area_by_color(img, color):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    ranges = hsv_ranges[color]
    mask = None
    if color == "red":
        mask1 = cv2.inRange(img_hsv, ranges['lower1'], ranges['upper1'])
        mask2 = cv2.inRange(img_hsv, ranges['lower2'], ranges['upper2'])
        mask = mask1 | mask2
    else:
        mask = cv2.inRange(img_hsv, ranges['lower'], ranges['upper'])
    # show_image(mask)
    color_only = cv2.bitwise_and(img, img, mask=mask)
    # show_image(color_only)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)  # remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)  # fill small holes
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    kept = []
    total_area = 0.0
    min_contour_area = 200
    for i in contours:
        area = cv2.contourArea(i)
        if area >= min_contour_area:
            kept.append(i)
            total_area += area
    color_only = cv2.bitwise_and(img, img, mask=mask)
    show_image(mask)
    show_image(color_only)
    out = img.copy()
    cv2.drawContours(out, kept, -1, (0, 0, 255), 2)
    show_image(out)
    return total_area


def main():
    # file_path = find_latest_image()
    img = cv2.imread("images/image5.jpg")
    # show_image(img)
    # blur_score = calculate_blur_score(img)
    # print("Laplacian variance blur score:", blur_score)
    # img = find_white_line(img)
    # show_image(img)
    print(calculate_area_by_color(img, "white"))


if __name__ == '__main__':
    main()
