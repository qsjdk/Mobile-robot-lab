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

     def laser_callback(self, scan_data):
         distances = [d if d > 0 else float('inf') for d in scan_data.ranges]
         min_dist = min(distances[-len(distances)/6:]+distances[:len(distances)/6])#只关注车前-60°到60°的视角
         min_index = distances.index(min_dist)
         angle = scan_data.angle_min + min_index * scan_data.angle_increment

         if angle > math.pi:
             angle -= 2 * math.pi

         pillar_x = min_dist * math.cos(angle)
         pillar_y = min_dist * math.sin(angle)
         rospy.loginfo(f"Pillar position: [{pillar_x}, {pillar_y}], Distance: {min_dist}, Angle: {angle}")

         self.adjust_heading(angle)
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
