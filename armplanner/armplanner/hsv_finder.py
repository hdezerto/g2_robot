import cv2
import numpy as np

# Load the image
img = cv2.imread('Pastedimage.png')  # Replace with your file
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Reshape and remove black pixels (optional, depends on your image background)
hsv_reshaped = hsv_img.reshape(-1, 3)
non_black = hsv_reshaped[np.any(hsv_reshaped > [0, 0, 0], axis=1)]  # Removes pure black pixels

# Get min and max HSV values
hsv_min = np.min(non_black, axis=0)
hsv_max = np.max(non_black, axis=0)

print(f"Lower HSV: {hsv_min}")
print(f"Upper HSV: {hsv_max}")