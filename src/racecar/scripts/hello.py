#!/usr/bin/env python3
import rospy
import math
import csv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

class LaserPointPublisher:
    def __init__(self):
        rospy.init_node('laser_point_pub', anonymous=True)
        self.pub = rospy.Publisher('/point_mid', Pose2D, queue_size=100)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.get_goal)
        self.new_laser = [0] * 600
        self.ind = [0] * 600
        self.last_point_x = 0
        self.last_point_y = 0

        self.clear_csv('/home/racecar/桌面/hello_ws/src/racecar/scripts/text.csv')

    def clear_csv(self, file_name):
        with open(file_name, 'w') as file_csv:
            file_csv.truncate()
        rospy.loginfo("CSV 文件已清空")

    def write_to_csv(self, x, y, angle, distance):
        with open('/home/racecar/桌面/hello_ws/src/racecar/scripts/text.csv', 'a', newline='') as file_csv:
            writer = csv.writer(file_csv)
            writer.writerow([x, y, angle, distance])
        rospy.loginfo("写入 CSV: X = %f, Y = %f, Angle = %f, Distance = %f", x, y, angle, distance)

    def bubble_sort(self, p, length, ind_diff):
        for m in range(length):
            ind_diff[m] = m
        for i in range(length):
            for j in range(length - i - 1):
                if p[j] > p[j + 1]:
                    p[j], p[j + 1] = p[j + 1], p[j]
                    ind_diff[j], ind_diff[j + 1] = ind_diff[j + 1], ind_diff[j]

    def get_goal(self, msg):
        if len(msg.ranges) < 1440:  # Check if there are enough ranges
            rospy.logwarn("激光数据长度不足: %d", len(msg.ranges))
            return

        num = 0
        for i in range(1140, 1441):
            self.new_laser[num] = msg.ranges[i]
            num += 1
        for i in range(301):
            self.new_laser[num] = msg.ranges[i]
            num += 1

        self.bubble_sort(self.new_laser, 600, self.ind)

        num_division = 1
        while num_division < 599 and abs(self.ind[num_division + 1] - self.ind[num_division]) <= 50:
            num_division += 1

        ob_1_angle = (self.ind[num_division] - 300) / 4
        ob_1_distance = self.new_laser[num_division]
        ob_2_angle = (self.ind[num_division + 1] - 300) / 4
        ob_2_distance = self.new_laser[num_division + 1]

        ob_1_x = ob_1_distance * math.cos(math.radians(ob_1_angle))
        ob_1_y = ob_1_distance * math.sin(math.radians(ob_1_angle))
        ob_2_x = ob_2_distance * math.cos(math.radians(ob_2_angle))
        ob_2_y = ob_2_distance * math.sin(math.radians(ob_2_angle))

        if ob_1_y != ob_2_y:
            point = Pose2D()
            point.x = (ob_1_x + ob_2_x) / 2
            point.y = (ob_1_y + ob_2_y) / 2
            point.theta = -(ob_1_x - ob_2_x) / (ob_1_y - ob_2_y)
            self.pub.publish(point)

            if math.hypot(point.x - self.last_point_x, point.y - self.last_point_y) >= 2.0:
                self.write_to_csv(point.x, point.y, ob_1_angle, ob_1_distance)
                self.last_point_x = point.x
                self.last_point_y = point.y

if __name__ == '__main__':
    try:
        LaserPointPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
