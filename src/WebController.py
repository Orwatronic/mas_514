#!/usr/bin/env python
"""
Node for hosting WebController application

Inspiration for this node:
https://medium.com/@aqib.mehmood80/wireless-video-surveillance-robot-using-raspberry-pi-9caadd3eb30b
"""
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mas514.msg import WebJoystick
import threading
from flask import Flask, render_template, Response, request
import time

# Start ROS node in seperate thread
threading.Thread(target=lambda: rospy.init_node('WebController', disable_signals=True)).start()

# Create a Flask Controller App
app = Flask(__name__)

class RosImage(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.frame = None

    def callback(self, msg):
        # Convert ROS image to bytes
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        self.frame = cv2.imencode('.jpg', cv_image)[1].tobytes()

    def gen(self):
        while True:
            if self.frame is not None:
                yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + self.frame + b'\r\n')

            time.sleep(0.05)
            
# Create RosImage class which handles the ROS image callabacks
rosImage = RosImage()
sub_markers = rospy.Subscriber("image_markers", Image, rosImage.callback)

# Joystick publishers
pub_joyLeft = rospy.Publisher('webJoystickLeft', WebJoystick, queue_size=1)
pub_joyRight = rospy.Publisher('webJoystickRight', WebJoystick, queue_size=1)



# Route creations
@app.route('/')
def index():
    """Home page."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    # return Response(gen(Camera()), mimetype='multipart/x-mixed-replace; boundary=frame')
    return Response(rosImage.gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/left_stick')
def left_stick():
    # Get joystick data
    joystick = WebJoystick()

    joystick.x = int(request.args.get('x'))
    joystick.y = int(request.args.get('y'))

    pub_joyLeft.publish(joystick)

    return render_template('index.html')

@app.route('/right_stick')
def right_stick():
    # Get joystick data
    joystick = WebJoystick()

    joystick.x = int(request.args.get('x'))
    joystick.y = int(request.args.get('y'))

    pub_joyRight.publish(joystick)

    return render_template('index.html')


if __name__ == '__main__':
    # Run Flask app
    wifi_ip = '192.168.137.91'
    app.run(host='0.0.0.0', port=8000, debug=True, use_reloader=False)
    




