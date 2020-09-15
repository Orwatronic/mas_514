# Including required packages
import os
from flask import Flask, render_template, Response, request
from numpy import interp
from camera_opencv import Camera
# Uncomment below line and comment above line if you want to use picamera package
# from camera_pi import Camera


direction = 'stop'

# Create a Flask Object called app
app = Flask(__name__)

def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
 
# Home page route
@app.route('/')
def index():
    """Home page."""
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Route to receive left joystick data
@app.route('/DCmotor')
def DCmotor():

    x = request.args.get('x_dist') # Get x-axis value
    y = request.args.get('y_dist') # Get y-axis value

    y = int(y)
    x = int(x)
    print(x, y)

    return render_template('index.html')

# Route to receive right joystick data
@app.route('/ServoMotor')
def ServoMotor():

    x = request.args.get('x_dist') # Get x-axis value
    y = request.args.get('y_dist') # Get y-axis value
    
    y = int(y)
    x = int(x)
    print(x, y)


    return render_template('index.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True, use_reloader=False)