#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class HomingController:
    def __init__(self):
        rospy.init_node('homing_controller')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.p_ang = 1.0
        self.p_vel = 1.2
        self.target_distance = 0.15
        self.twist_msg = Twist()
        
        rospy.loginfo("Homing Controller Initialized")

    # def laser_callback(self, scan_data):
    #     distances = [d if d > 0 else float('inf') for d in scan_data.ranges]
    #     min_dist = min(distances)
    #     min_index = distances.index(min_dist)
    #     angle = scan_data.angle_min + min_index * scan_data.angle_increment

    #     if angle > math.pi:
    #         angle -= 2 * math.pi

    #     pillar_x = min_dist * math.cos(angle)
    #     pillar_y = min_dist * math.sin(angle)
    #     rospy.loginfo(f"Pillar position: [{pillar_x}, {pillar_y}], Distance: {min_dist}, Angle: {angle}")

    #     self.adjust_heading(angle)
    #     self.adjust_speed(min_dist)
    #     self.drive_robot()

    def laser_callback(self, scan_data):
        threshold_diff = 1  # 跳变距离阈值（大1米）
        left_boundary_index = None
        right_boundary_index = None
        pillar_angle = 0
        min_dist = 0

        distances = [d if d > 0 else float('inf') for d in scan_data.ranges]

        # 遍历激光数据，寻找左右边界
        for i in range(1, len(distances) - 1):
            current_dist = distances[i]

            # 过滤掉无效值
            if current_dist <= 0 or current_dist == float('inf'):
                continue

            # 找到左边界：右侧距离比当前点大于阈值
            if distances[i + 1] > current_dist + threshold_diff:
                left_boundary_index = i if left_boundary_index is None else left_boundary_index

            # 找到右边界：左侧距离比当前点大于阈值
            if distances[i - 1] > current_dist + threshold_diff:
                right_boundary_index = i

            # 当左右边界都找到时，跳出循环
            if left_boundary_index is not None and right_boundary_index is not None:
                break

        # 检查是否找到左右边界
        if left_boundary_index is not None and right_boundary_index is not None:
            # 计算柱子的角度为左右边界的中点
            middle_index = (left_boundary_index + right_boundary_index) // 2
            min_dist = distances[middle_index]
            pillar_angle = scan_data.angle_min + middle_index * scan_data.angle_increment

            # 将角度转换到 [-π, π] 范围
            if pillar_angle > math.pi:
                pillar_angle -= 2 * math.pi

            # 输出柱子位置
            pillar_x = min_dist * math.cos(pillar_angle)
            pillar_y = min_dist * math.sin(pillar_angle)
            rospy.loginfo(f"Pillar position: [{pillar_x}, {pillar_y}], Distance: {min_dist}, Angle: {pillar_angle}")

            
        else:
            rospy.logwarn("Pillar boundaries not found within expected range.")

        # 调整方向并控制速度
        self.adjust_heading(pillar_angle)
        self.adjust_speed(min_dist)
        self.drive_robot()

    def adjust_heading(self, angle):
        self.twist_msg.angular.z = self.p_ang * angle

    def adjust_speed(self, distance):
        if distance > self.target_distance:
            self.twist_msg.linear.x = min(self.p_vel * (distance - self.target_distance), 5.0)
        else:
            self.twist_msg.linear.x = 0
            self.twist_msg.angular.z = 0

    def drive_robot(self):
        self.vel_pub.publish(self.twist_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    controller = HomingController()
    controller.run()