# 使用指南

1.解压压缩包，进去catkin_path_tracking工作目录 2.catkin_make 编译

## Purepursuit算法启动

1. roslaunch gazebo_ros empty_world.launch 启动gazebo空白世界
2. roslaunch classb_car_model default_car.launch 启动小车模型并加载到gazebo仿真环境中
3. roslaunch purepursuit_pkg pp_start.launch 启动Purepursuit算法

## Stanley算法启动

1. roslaunch gazebo_ros empty_world.launch 启动gazebo空白世界
2. roslaunch classb_car_model default_car.launch 启动小车模型并加载到gazebo仿真环境中
3. roslaunch stanley_pkg stanley_start.launch 启动Stanley算法

LQR、MPC等算法后续更新