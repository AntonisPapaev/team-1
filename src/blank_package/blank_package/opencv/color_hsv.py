import numpy as np

hsv_ranges = {

    "red": {
        "lower1": np.array([0, 120, 70]),
        "upper1": np.array([10, 255, 255]),
        "lower2": np.array([170, 120, 70]),
        "upper2": np.array([179, 255, 255])
    },

    "orange": {
        "lower": np.array([11, 120, 70]),
        "upper": np.array([25, 255, 255])
    },

    "yellow": {
        "lower": np.array([26, 120, 70]),
        "upper": np.array([35, 255, 255])
    },

    "green": {
        "lower": np.array([36, 80, 70]),
        "upper": np.array([85, 255, 255])
    },

    "blue": {
        "lower": np.array([90, 80, 70]),
        "upper": np.array([130, 255, 255])
    },

    "purple": {
        "lower": np.array([131, 80, 70]),
        "upper": np.array([160, 255, 255])
    },

    "white": {
        "lower": np.array([0, 0, 200]),
        "upper": np.array([179, 40, 255])
    },

    "black": {
        "lower": np.array([0, 0, 0]),
        "upper": np.array([179, 255, 50])
    },

    "grey": {
        "lower": np.array([0, 0, 50]),
        "upper": np.array([179, 40, 200])
    }
}
