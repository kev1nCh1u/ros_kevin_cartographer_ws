# cartographer_2d_lidar_ws


## cartographer
    catkin_make_isolated --install --use-ninja
    source devel_isolated/setup.bash
## 存地圖指令
    rosservice call /finish_trajectory 0

    rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"

## rosrun
    rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/mymap -pbstream_filename=${HOME}/Downloads/mymap.pbstream -resolution=0.05
