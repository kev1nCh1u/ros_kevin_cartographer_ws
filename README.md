# ros_kevin_cartographer_ws


## kevin 2d
    roslaunch urg_node urg_lidar.launch
    roslaunch launch_start my_robot_2d.launch

## kevin 3d
    roslaunch velodyne_pointcloud VLP16_points.launch

    roslaunch vmu931 vmu_read_all.launch

    roslaunch launch_start my_robot_3d.launch

## create map
    roslaunch launch_start hokuyo_2d.launch

## load map
    roslaunch launch_start tutorial.launch
## cartographer build
    catkin_make_isolated --install --use-ninja
    source devel_isolated/setup.bash
## 存地圖指令
    rosservice call /finish_trajectory 0

    rosservice call /write_state "{filename: '${HOME}/Downloads/mymap.pbstream'}"

## pbstream to rosmap
    rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Downloads/mymap -pbstream_filename=${HOME}/Downloads/mymap.pbstream -resolution=0.05


## ex demo
    roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=/home/user/rosbags/cartographer/b3-2016-04-05-13-54-42.bag

## velodyne

### ip 192.168.1.201

    roslaunch velodyne_pointcloud VLP16_points.launch

    rosrun rviz rviz -f velodyne

    rosrun rviz rviz -d /home/user/ros/kevin_cartographer_ws/src/vlp_20210223.rviz