# Imports
import cv2
import pathlib
import numpy as np

# Set a folder for saving images
imageLocation = pathlib.Path("./images")

# Shape of chessboard
sh1 = 6
sh2 = 8

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Make object point arrays, used by camera calibration function
objp = np.zeros((sh1 * sh2, 3), np.float32)
objp[:, :2] = np.mgrid[0:sh1, 0:sh2].T.reshape(-1, 2)

# Initialize list
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# Find all images in path
filenames = imageLocation.glob("*.png")
for filename in filenames:
    # Read image
    img = cv2.imread(str(filename))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find chess board corners
    ret, corners = cv2.findChessboardCorners(gray, patternSize=(sh1, sh2))

    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
    else:
        print("Image %s trashed" % (str(filename)))

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# Save calibration data
calibrationPath = imageLocation.joinpath("intrinsicCalibration.npz")
np.savez(str(calibrationPath), mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
