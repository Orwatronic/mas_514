<launch>
<node name="Odometry" pkg="mas514" type="Odometry.py" output="screen"/>
    <node name="serial_node" pkg="rosserial_python" type="serial_node.py" output="screen">
    <param name="port" value="/dev/ttyUSB0"/>
    <param name="baud" value="115200"/>
  </node>
    <!-- <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" >
        <arg name="port" value="9090"/>
    </include> -->
  <!-- <node name="InverseKinematics" pkg="mas514" type="InverseKinematics.py" output="screen"/>  -->
</launch>
