source ../devel/setup.bash
export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/my_robot/worlds/jonathan.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/my_robot/maps/map.yaml"
export ROBOT_INITIAL_POSE="-x 0 -y 0 -z 0"
xterm -e "roslaunch turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch" &
sleep 7
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"  &
sleep 6
xterm -e "roslaunch turtlebot_navigation amcl_demo.launch initial_pose_a:=2 map_file:=$TURTLEBOT_GAZEBO_MAP_FILE" &
sleep 8
xterm -e "roslaunch add_markers add_markers.launch" &
sleep 1
xterm -e "roslaunch pick_objects pick_objects.launch" &
echo $TURTLEBOT_GAZEBO_MAP_FILE
