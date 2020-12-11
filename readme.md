## Creating the map

1.  roslaunch tobor_gazebo tobor_world.launch world:=<name of world>
2.  roslaunch tobor_navigation tobor_slam.launch
3.  Drive robot around
    
### Run the joystick 
1.  rosrun teleop_twist_joy teleop_node
2.  Rosrun joy joy_node
3.  Hold down button 0
    
## Save the Map
1. rosrun map_server map_saver -f ~/tobor_ws/src/tobor_navigation/maps/<name of map>

## Running Everything
1. roslaunch tobor_gazebo tobor_world.launch world:=<name of world>
2. roslaunch tobor_navigation tobor_navigation.launch map_name:=<name of map>
-   Running without particle filter use “use_pf:=false”
