import cv2
import pathlib
import numpy as np

# Set location of all image
path = pathlib.Path(r'./whiteimages')

# Set size of images in pixels
width = 720
height = 540

# Full path of all images within folder
filenames = [x for x in path.glob('*.png') if x.is_file()]

# Number of samples
N = len(filenames)

# Init placeholders
images = np.zeros((height, width, 3, N), dtype=np.uint8)
gains = np.zeros((height, width, 3), float)
means = np.zeros(N, float)
v = np.zeros((N), float)

# Iterate thorugh all images and save to arrays
i = 0
for filename in filenames:
    img = cv2.imread(str(filename))
    means[i] = img.mean()
    images[:, :, :, i] = img
    i += 1

# Calculate the calibration matrix
for i in range(0, images.shape[0]):  # height-direction
    for j in range(0, images.shape[1]):  # width-direction
        for k in range(0, 3): # color
            v[:] = images[i, j, k, :]

            # Ordinary Least Squares
            temp1 = np.sum(v**2)
            temp2 = np.sum(v*means)
            gains[i, j, k] = temp1**-1 * temp2

# Save color calibration gains
np.savez(str(path.joinpath('colorCalibration.npz')), arr_0=gains)
