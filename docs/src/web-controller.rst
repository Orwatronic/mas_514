##############
Web Controller
##############

Actions done:
- Add folders static and templates to ./src
- Add WebController.py to ./src and sudo chmod +x WebController.py
- Add WebJoystick.msg to ./msg
- Add WebJoystick.msg in CMakeLists.txt under add_message_files()
- Execute cakin_make from ~/catkin_ws folder
- Add <node name="WebController" pkg="mas507" type="WebController.py" output="screen"/> to start.launch file
- Visit http://jetbot-desktop<group-number>:8000