https://github.com/LCAS/CMP9767_LIMO/wiki

cd <to kasm-lcas-jammy.docker-compose.yaml>
docker login lcas.lincoln.ac.uk -<login> -p <pass>

docker compose -f kasm-lcas-jammy.docker-compose.yaml pull
docker compose -f kasm-lcas-jammy.docker-compose.yaml up -d

https://localhost:6901
kasm_user
password


ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
. install/setup.bash 
or /opt/ros/lcas_addons/install/setup.bash




-------

• colcon is the name of the ROS2 build system
• To create a new workspace, first we have to source ROS to bash, the linux shell:
• source /opt/ros/humble/setup.bash
• To make a catkin workspace and initialize it:
•mkdir –p ~/catkin_ws/src
• cd ~/catkin_ws/src
• colcon build
• To create a new package:
•cd ~/catkin_ws/src
•catkin_create_pkg my_package_name rospy...
• Point to note, always source bash when in new terminal by:
• source install/setup.bash