export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/my_robot/worlds/jonathan.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/my_robot/maps/map.yaml"
xterm -e "roslaunch turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch" &
xterm -e "roslaunch turtlebot_navigation gmapping_demo.launch" &
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
