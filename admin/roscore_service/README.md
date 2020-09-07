# Inspiration
https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/

# Installation
1) Move `roscore_service` to `~/catkin_ws/src`  folder
1) Change `counter.py` to executable by using `chmod +x counter.py` inside the `src` folder in the `roscore_service` package
1) Install robot_upstart as: `rosrun robot_upstart install roscore_service/launch/start.launch --job jetbot_ros --symlink`
1) Start system service `sudo systemctl daemon-reload && sudo systemctl start jetbot_ros`
1) Test counter output (should print an incresing number) `rostopic echo /counter`

# Check Status for Service
`sudo systemctl status jetbot_ros`

