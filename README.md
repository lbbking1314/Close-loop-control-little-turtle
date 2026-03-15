# Close-loop-control-little-turtle
闭环轨迹控制小海龟
# 终端1：启动ROS核心
roscore

# 终端2：启动小海龟仿真器
rosrun turtlesim turtlesim_node

# 终端3：运行闭环控制节点（保存代码为turtle_square_closedloop.py）
chmod +x turtle_square_closedloop.py  # 添加执行权限
python3 turtle_square_closedloop.py   # 直接运行

实现思路
闭环控制核心逻辑：
发布速度指令（/turtle1/cmd_vel）→ 订阅位姿反馈（/turtle1/pose）→ 根据当前位姿与目标位姿的偏差调整指令；
正方形轨迹规划：
分 4 段直线运动，每段目标：沿当前方向走固定距离（如 2m），然后转向 90°；
通过位姿计算已行走距离和当前角度，当偏差小于阈值时完成该段运动。
