# 起硬件服务进程
sudo /etc/Haption/SvcHaptic_virtuose_6d_n190

# ROS2功能包依赖
source install/setup.bash

# 启动硬件驱动主节点 起server
ros2 run virtuose_ros2 virtuose_node 
