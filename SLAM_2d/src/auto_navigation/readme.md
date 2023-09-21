<!-- 
*author: 郭嘉豪
*Time: 2023-3
*version: 1.0 
-->
# 使用说明

## 单独调试启动
单独调试导航包，需要启动launch/robot_start.launch,发布里程计和雷达信息

首次建图：

- pc_launch/nav_exploring_slam_demo.launch 可以获取上面的信息进行：建图+导航（建图只需要一次，之后只需启动下面的launch即可）
建图完成后之后直接启动下列任意一个即可

- pc_launch/nav_map_amcl_rviz.launch  可以获取最上面的信息进行：可以加载上一个建好的图+导航（手动rviz内发布目标点）

- pc_launchnav_map_amcl_Auto.launch 可以获取最上面的信息进行：可以加载上一个建好的图+导航+自动巡逻（也可以往/goal_pose话题内直接输入目标点打断当前巡逻，超时后自动继续巡逻）

## auto_navigation说明
- src/publish_node.cpp 相当于main,完成了串口消息和导航的消息订阅，进行里程计计算

- src/robot_start.cpp  包含机器人里程计计算函数，位姿计算函数，是对底盘的实时信息处理到适合ros消息的地方

## 调试说明
- nav_pub_initpose.cpp ：设置机器人初始姿态的文件和节点，需要对应地图和摆放位置设置

- nav_auto.py ： 设置几个定点可以使得机器人在几个点之间自动寻路

- move_base_TebPlanner.launch ： 调试机器人本地规划参数，可以使用ros自带的可视化调参工具

- pc_launch、launch本质没有任何区别，只是以前调试时，笔记本作为ros主机启动pc_launch内的文件，launch只在车载pc内启动发布雷达和里程计，现在哨兵内启动时都部署在车载pc内，所以已经无区别