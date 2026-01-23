#!/usr/bin/python3
import cv2
import os
import re
import numpy as np
from opencv.color_hsv import hsv_ranges


# import matplotlib.pyplot as plt


def find_latest_image():
    folder = "images"
    pattern = re.compile(r"(\d+)\.jpg")

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


class Image:
    def __init__(self, img):
        self.img = img

    def show_image(self, img):
        cv2.imshow("img", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def calculate_blur_score(self):
        grey = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        blur_score = cv2.Laplacian(grey, cv2.CV_64F)
        return blur_score.var()

    def find_white_line(self):
        blur = cv2.GaussianBlur(self.img, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)  # within the range = white, everything else = black
        edges = cv2.Canny(mask, 50, 150)  # makes edge pixels white
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        # makes an array of line segments - shape (N, 1, 4)
        out = self.img.copy()

        if lines is not None:
            for (x1, y1, x2, y2) in lines[:, 0]:  # (N, 1, 4) -> (N, 4)
                cv2.line(out, (x1, y1), (x2, y2), (0, 0, 255), 2)
        return out

    def calculate_area_by_color(self, color):
        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        ranges = hsv_ranges[color]
        mask = None
        if color == "red":
            mask1 = cv2.inRange(img_hsv, ranges['lower1'], ranges['upper1'])
            mask2 = cv2.inRange(img_hsv, ranges['lower2'], ranges['upper2'])
            mask = mask1 | mask2
        else:
            mask = cv2.inRange(img_hsv, ranges['lower'], ranges['upper'])
        # show_image(mask)
        color_only = cv2.bitwise_and(self.img, self.img, mask=mask)
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
        color_only = cv2.bitwise_and(self.img, self.img, mask=mask)
        self.show_image(mask)
        self.show_image(color_only)
        out = self.img.copy()
        cv2.drawContours(out, kept, -1, (0, 0, 255), 2)
        self.show_image(out)
        return total_area

    def crop_bottom_half(self, image):
        return image[image.shape[0] // 2:]

    def find_mid_black_line(self):
        self.img = self.crop_bottom_half(self.img)
        blur = cv2.GaussianBlur(self.img, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        # hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
        # self.show_image(hsv)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([179, 190, 90])
        mask = cv2.inRange(hsv, lower_black, upper_black)  # within the range = white, everything else = black
        edges = cv2.Canny(mask, 50, 150)  # makes edge pixels white
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # makes an array of line segments - shape (N, 1, 4)

        out = self.img.copy()
        cv2.drawContours(out, contours, -1, (0, 0, 255), 2)

        # self.show_image(out)
        height, width, channels = self.img.shape
        h1 = height / 4
        h2 = height * 3 / 4
        pred_x1_array = []
        pred_x2_array = []
        # print(h1, h2)
        # print(lines)
        if lines is None:
            return -1, -1, -1, -1, -1, -1
        for i in lines[:, 0]:
            # print(i)
            x1, y1, x2, y2 = i[0], i[1], i[2], i[3]
            if x2 != x1:
                m = (y2 - y1) / (x2 - x1)
                if m != 0:
                    pred_x1 = (h1 - y1) / m + x1
                    pred_x2 = (h2 - y1) / m + x1
                    pred_x1_array.append(pred_x1)
                    pred_x2_array.append(pred_x2)
                    # print(pred_x1)
        #     length = ((y2-y1)**2+(x2-x1)**2)**0.5
        #     print(length)
        #     x_coord = np.array([x1, x2])
        #     y_coord = np.array([y1, y2])
        #     plt.plot(x_coord, y_coord, color="black")
        pred_x1_array = np.array(pred_x1_array)
        pred_x2_array = np.array(pred_x2_array)
        mid_x1 = pred_x1_array.mean()
        mid_x2 = pred_x2_array.mean()
        # x_coord = np.array([mid_x1, mid_x2])
        # y_coord = np.array([h1, h2])
        # plt.plot(x_coord, y_coord, color="red")
        # plt.show()
        return mid_x1, mid_x2, h1, h2, height, width

    def find_error_from_middle(self):

        img = self.img
        if img is None:
            return 0.0, False, 0

        h, w = img.shape[:2]

        # ROI: bottom half
        roi = img[h // 2 : h, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Black threshold (may need small tuning per lighting)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([179, 190, 90])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Clean noise (keep light – too much close merges blobs)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        black_pixels = int(cv2.countNonZero(mask))
        if black_pixels == 0:
            return 0.0, False, black_pixels

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, False, black_pixels

        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        top = contours[:6]  # try a few, filtering will pick the true lines

        line_xs = []
        roi_h, roi_w = mask.shape[:2]
        y_pick = int(roi_h * 0.85)

        for c in top:
            area = cv2.contourArea(c)
            if area < 200:
                continue

            x, y, cw, ch = cv2.boundingRect(c)

            # reject blobs / shadows / road
            if ch < 25:
                continue
            if cw > 80:
                continue
            if (ch / float(cw + 1e-6)) < 1.2:
                continue

            pts = c[:, 0, :]
            ys = pts[:, 1]
            xs = pts[:, 0]

            bottom_xs = xs[ys >= y_pick]
            if bottom_xs.size == 0:
                continue

            line_xs.append(float(np.mean(bottom_xs)))

        if len(line_xs) < 2:
            return 0.0, False, black_pixels

        line_xs.sort()
        lane_center_x = (line_xs[0] + line_xs[1]) / 2.0

        cx = roi_w / 2.0
        pixel_error = lane_center_x - cx
        norm_error = float(pixel_error / cx)

        # clamp
        if norm_error < -1.0:
            norm_error = -1.0
        elif norm_error > 1.0:
            norm_error = 1.0

        return norm_error, True, black_pixels
# def main():
#     # file_path = find_latest_image()
#     img = cv2.imread("images/image22.png")
#     image = Image(img)
#     image.show_image(img)
#     # blur_score = calculate_blur_score(img)
#     # print("Laplacian variance blur score:", blur_score)
#     # img = find_white_line(img)
#     # show_image(img)
#     # print(calculate_area_by_color(img, "white"))
#     print(image.find_error_from_middle())
#
#
# if __name__ == '__main__':
#     main()
