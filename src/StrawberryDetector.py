import cv2
from cv_bridge import CvBridge
import numpy as np

# Inspired from: https://github.com/andridns/cv-strawberry
class StrawberryDetector(object):
    def __init__(self, ros_publisher, mtx, dist):
        self.cvBridge = CvBridge()
        self.ros = ros_publisher
        self.mtx = mtx
        self.dist = dist
        self.x = 0
        self.y = 0
        self.z = 0


    def callback(self, msg):
        # Read image message and convert to CV image
        image = self.cvBridge.imgmsg_to_cv2(msg)

        # Convert from BGR to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Blur image slightly
        image_blur = cv2.GaussianBlur(image, (7, 7), 0)

        # Convert to HSV
        image_blur_hsv = cv2.cvtColor(image_blur, cv2.COLOR_RGB2HSV)

        # 0-10 hue
        min_red = np.array([0, 100, 80])
        max_red = np.array([10, 256, 256])
        image_red1 = cv2.inRange(image_blur_hsv, min_red, max_red)

        # 170-180 hue
        min_red2 = np.array([170, 100, 80])
        max_red2 = np.array([180, 256, 256])
        image_red2 = cv2.inRange(image_blur_hsv, min_red2, max_red2)

        # Create red image
        image_red = image_red1 + image_red2

        # Fill small gaps and remove specks
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        image_red_closed = cv2.morphologyEx(image_red, cv2.MORPH_CLOSE, kernel)
        image_red_closed_then_opened = cv2.morphologyEx(image_red_closed, cv2.MORPH_OPEN, kernel)

        # Find biggest red countour
        big_contour, red_mask = self.find_biggest_contour(image_red_closed_then_opened)

        if (big_contour is not None) and (red_mask is not None):
            # Centre of mass
            moments = cv2.moments(red_mask)
            centre_of_mass = int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])

            # Add poitn of COM in image
            cv2.circle(image, centre_of_mass, 5, (0, 255, 0), -1, cv2.LINE_AA)

            # Fit ellipse to detection
            ellipse = cv2.fitEllipse(big_contour)
            cv2.ellipse(image, ellipse, (0, 255, 0), 1)

            # Strawberry radius based on ellipse fitting
            r = int(ellipse[1][0]/2)

            # Add circle to image around COM and image boundary using calculated radius
            cv2.circle(image, centre_of_mass, r, (0, 0, 255), 1, cv2.LINE_AA)
            
            # PnP for straberry location
            n = 30
            t = np.linspace(0, 2*np.pi, n)

            # 3D points of strawaberry major circkle
            radius_strawberry = (32.0/1000.0)/2
            objpnts = np.zeros([n,3])
            objpnts[:,0] = radius_strawberry*np.sin(t)
            objpnts[:,1] = radius_strawberry*np.cos(t)

            # 2D points in image of detected strawberry major circle
            imgpoints = np.zeros([n,2])
            imgpoints[:,0] = centre_of_mass[0] + r*np.sin(t)
            imgpoints[:,1] = centre_of_mass[1] + r*np.cos(t)
            
            # Solve for position and oreintation
            _, rvec, tvec = cv2.solvePnP(objpnts, imgpoints, self.mtx, self.dist)
            
            # Return detection results to class
            self.x = tvec[0]
            self.y = tvec[1]
            self.z = tvec[2]


        # Publish image with detection
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.ros.publish(self.cvBridge.cv2_to_imgmsg(image_bgr, "bgr8"))
        
        
    def find_biggest_contour(self, image):
        # Copy to prevent modification
        image = image.copy()
        contours, hierarchy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Isolate largest contour
        contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]

        # Return results
        if len(contour_sizes) != 0:
            biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
    
            mask = np.zeros(image.shape, np.uint8)
            cv2.drawContours(mask, [biggest_contour], -1, 255, -1)

        else:
            biggest_contour = None
            mask = None

        return biggest_contour, mask







