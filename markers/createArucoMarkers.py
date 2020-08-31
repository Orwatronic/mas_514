import cv2
from cv2 import aruco
import matplotlib.pyplot as plt

# Select Aruco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Create markers
for i in range(0, 3):
    img = aruco.drawMarker(aruco_dict, i, sidePixels=700)

    cv2.imwrite('id' + str(i) + '.png', img)







