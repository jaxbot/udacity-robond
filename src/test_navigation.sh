source ../devel/setup.bash
export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/my_robot/worlds/jonathan.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/my_robot/maps/map.yaml"
export ROBOT_INITIAL_POSE="-x 2 -y 0 -z 0"
#xterm  -e  " gazebo $TURTLEBOT_GAZEBO_WORLD_FILE" &
xterm -e "roslaunch turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"  &
sleep 5
xterm -e "roslaunch turtlebot_navigation amcl_demo.launch map_file:=$TURTLEBOT_GAZEBO_MAP_FILE" &
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
echo $TURTLEBOT_GAZEBO_MAP_FILE
