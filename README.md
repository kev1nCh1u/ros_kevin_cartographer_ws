# cartographer_2d_lidar_ws


## kevin
    roslaunch urg_node urg_lidar.launch
    roslaunch launch_start my_robot_2d.launch

## create map
    roslaunch launch_start hokuyo_2d.launch

## load map
    roslaunch launch_start hokuyo_2d.launch
## cartographer build
    catkin_make_isolated --install --use-ninja
    source devel_isolated/setup.bash
## 存地圖指令
    rosservice call /finish_trajectory 0

    rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"

## pbstream to rosmap
    rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/mymap -pbstream_filename=${HOME}/Downloads/mymap.pbstream -resolution=0.05
