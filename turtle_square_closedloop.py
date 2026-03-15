#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS闭环轨迹控制：小海龟走正方形（订阅位姿反馈，精准控制）
"""
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# 全局变量存储当前位姿
current_pose = Pose()
# 标记位姿是否已更新
pose_updated = False


def pose_callback(pose_msg):
    """
    位姿订阅回调函数：更新当前小海龟的位姿（x,y,θ）
    """
    global current_pose, pose_updated
    current_pose = pose_msg
    pose_updated = True


def move_straight(vel_pub, target_distance, linear_speed=0.5):
    """
    闭环直行控制：根据位姿反馈，精准走指定距离
    :param vel_pub: 速度发布者对象
    :param target_distance: 目标直行距离 (m)
    :param linear_speed: 直行线速度 (m/s)
    """
    global current_pose, pose_updated
    vel_msg = Twist()

    # 记录起始位姿（x,y）
    start_x = current_pose.x
    start_y = current_pose.y
    rospy.loginfo(f"开始直行，目标距离：{target_distance}m")

    # 闭环控制循环：直到行走距离达到目标
    while not rospy.is_shutdown():
        # 确保位姿已更新
        if not pose_updated:
            continue

        # 计算已行走的欧氏距离（闭环反馈核心）
        distance_traveled = math.hypot(
            current_pose.x - start_x,
            current_pose.y - start_y
        )

        # 距离偏差判断：达到目标则停止
        if distance_traveled >= target_distance:
            rospy.loginfo(f"直行完成，实际行走：{distance_traveled:.2f}m")
            break

        # 发布直行指令（x轴正方向）
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = 0.0
        vel_pub.publish(vel_msg)

        # 控制发布频率
        rospy.Rate(10).sleep()

    # 停止直行
    vel_msg.linear.x = 0.0
    vel_pub.publish(vel_msg)


def rotate(vel_pub, target_angle, angular_speed=0.5):
    """
    闭环转向控制：根据位姿反馈，精准转向指定角度（相对角度）
    :param vel_pub: 速度发布者对象
    :param target_angle: 目标转向角度 (°，正值左转，负值右转)
    :param angular_speed: 转向角速度 (rad/s)
    """
    global current_pose, pose_updated
    vel_msg = Twist()

    # 转换角度单位：角度→弧度
    target_angle_rad = math.radians(target_angle)
    # 记录起始角度
    start_angle = current_pose.theta
    # 目标绝对角度（处理弧度循环：-π~π）
    target_abs_angle = math.fmod(start_angle + target_angle_rad, 2 * math.pi)
    if target_abs_angle > math.pi:
        target_abs_angle -= 2 * math.pi
    elif target_abs_angle < -math.pi:
        target_abs_angle += 2 * math.pi

    rospy.loginfo(f"开始转向，目标角度：{target_angle}°")

    # 闭环控制循环：直到角度偏差小于阈值（0.01rad≈0.57°）
    while not rospy.is_shutdown():
        if not pose_updated:
            continue

        # 计算角度偏差（闭环反馈核心）
        angle_error = target_abs_angle - current_pose.theta
        # 处理角度循环偏差（如从350°转到10°，偏差应为20°而非-340°）
        angle_error = math.fmod(angle_error + math.pi, 2 * math.pi) - math.pi

        # 角度偏差小于阈值则停止
        if abs(angle_error) < 0.01:
            rospy.loginfo(f"转向完成，实际偏差：{math.degrees(angle_error):.2f}°")
            break

        # 根据偏差发布转向指令（保证转向方向正确）
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = angular_speed if angle_error > 0 else -angular_speed
        vel_pub.publish(vel_msg)

        rospy.Rate(10).sleep()

    # 停止转向
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)


def turtle_square_closedloop():
    """
    主函数：闭环控制小海龟走正方形
    """
    # 1. 初始化ROS节点
    rospy.init_node('turtle_square_closedloop_controller', anonymous=True)

    # 2. 创建速度发布者
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # 3. 创建位姿订阅者（核心：获取闭环反馈）
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    # 等待位姿订阅生效（避免初始位姿未更新）
    rospy.sleep(1)
    if not pose_updated:
        rospy.logerr("未接收到小海龟位姿数据！")
        return

    # ========== 闭环控制参数 ==========
    square_side = 2.0  # 正方形边长 (m)
    turn_angle = 90.0  # 每次转向角度 (°)
    linear_speed = 0.5  # 直行速度 (m/s)
    angular_speed = 0.5  # 转向速度 (rad/s)

    rospy.loginfo("闭环控制启动：小海龟将走正方形 🟥")
    rospy.loginfo(f"参数：边长={square_side}m，转向角度={turn_angle}°")

    try:
        # 循环4次：直行+转向（正方形4条边）
        for i in range(4):
            rospy.loginfo(f"\n===== 第{i + 1}条边 =====")
            # 闭环直行：走指定边长
            move_straight(vel_pub, square_side, linear_speed)
            # 闭环转向：转90°
            rotate(vel_pub, turn_angle, angular_speed)

        # 完成轨迹后停止
        rospy.loginfo("✅ 正方形轨迹完成，小海龟已停止")

    except rospy.ROSInterruptException:
        rospy.logwarn("⚠️ 节点被中断，小海龟已紧急停止")
        # 发布停止指令
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        vel_pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        turtle_square_closedloop()
    except rospy.ROSInterruptException:
        pass