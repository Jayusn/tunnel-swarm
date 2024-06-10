# vins_to_mavros

#### 介绍
VINS里程计传给mavros的功能包

#### 使用说明

- 直接clone到你的工作空间的src下，这是一个功能包而以
- 启动好VINS，并设置好ekf2 aid mask和ekf2 hgt mode参数
- 输入rosrun vins_to_mavros vins_to_mavros_node 
- 如果用激光雷达的里程计，那么把cpp里的话题换成雷达的odom就行
- 还有一点就是，确保你启动mavros后有mavros/vision_pose/pose这个话题，如果没有的话，说明mavros没有下全，需要输入：sudo apt install ros-noetic-mavros-extras

