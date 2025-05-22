#!/usr/bin/env python3
#coding:utf-8
from geometry_msgs.msg import PoseStamped, Pose
import rospy
import string
import math
import time
import sys
import csv
import os
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

#repeat the route use for test 
#功能:循环导航 
#csv要求 一个第点是起点下一个点，终点是起点 1230
class MultiGoals:
    def __init__(self, goalListX, goalListY, retry, map_frame):
        self.retry = 1
        if(self.retry == 1):
            #self.sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=1)
            self.pub = rospy.Publisher(
                'move_base_simple/goal', PoseStamped, queue_size=10)
            #self.pose_ekf = rospy.Subscriber('/odometry/filtered',Odometry,self.getPose_ekf,queue_size=10)
            self.pose_amcl = rospy.Subscriber(
                "/amcl_pose", PoseWithCovarianceStamped, self.getPose_amcl, queue_size=10)
            # params & variables
            self.pub_final = rospy.Publisher('/arrfinal', Float64, queue_size=1)
            self.goalListX = goalListX
            self.goalListY = goalListY
            self.goalListW = goalListW
            self.goalListZ = goalListZ
            self.kx = 0
            self.ky = 0
            self.gx = 0
            self.gy = 0
            self.flag = 1
            self.MIN_DISTANCE = 1.3 # 4 min distance of the judge between the goal and odometrypose
            self.LONG = len(self.goalListX)
            self.goalId = 0
            self.count = 0
            self.start_time = 0
            self.pubfinal = False
            self.goalMsg = PoseStamped()
            self.goalMsg.header.frame_id = map_frame

            time.sleep(1)
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.pub.publish(self.goalMsg)
            # print(self.goalMsg)
            self.start_time = rospy.get_time()
            rospy.loginfo(
                "Initial goal published! Goal ID is: %d", self.goalId)
            self.goalId = self.goalId + 1

    def statusCB(self):


        self.gx = self.goalListX[self.goalId-1] if(self.goalId != 0) else self.goalListX[self.goalId]
        self.gy = self.goalListY[self.goalId-1] if(self.goalId != 0) else self.goalListY[self.goalId]

        self.dist = self.distance(self.kx, self.ky, self.gx, self.gy)

        if self.dist < self.MIN_DISTANCE and self.flag == 1:

            finish_time = rospy.get_time()
            interval = finish_time - self.start_time
            print(interval)
            if self.goalId == self.LONG:
                pass

            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.pub.publish(self.goalMsg)
            rospy.loginfo(
                "Initial goal published! Goal ID is: %d", self.goalId)
            rospy.loginfo("intostatusCB")

            if self.goalId == 1 or self.goalId == 15 or self.goalId == 26:
                self.pub_final.publish(1.0)
            if self.goalId == 23:
                self.pub_final.publish(2.0)               
                self.MIN_DISTANCE = 1.35
            if self.goalId == 25:
                self.MIN_DISTANCE = 1.7
            # # if self.goalId == 25:
            # #     self.MIN_DISTANCE = 0.6
            if self.goalId == 26:
                self.MIN_DISTANCE = 0.5
            # if self.goalId == 27:
            #     self.pub_final.publish(2.0) 
            self.count = self.count+1

            print(self.count)

            if self.goalId < (len(self.goalListX))-1:
                self.goalId = self.goalId + 1

            else:
                self.pub_final.publish(1.0) 
                print("final")

    def getPose_ekf(self, data):
        self.kx = data.pose.pose.position.x
        self.ky = data.pose.pose.position.y
        self.statusCB()

    def getPose_amcl(self, data):
        self.kx = data.pose.pose.position.x
        self.ky = data.pose.pose.position.y
        self.statusCB()

    def distance(self, kx, ky, gx, gy):
        try:
            return math.sqrt((kx-gx)**2+(ky-gy)**2)
        except:
            return None



if __name__ == "__main__":
    try:
        # ROS Init
        rospy.init_node('multi_goals', anonymous=True)
        retry = 1
        goalList = []
        goalListX=[]
        goalListY=[]
        goalListZ=[]
        goalListW=[]
        map_frame = rospy.get_param('~map_frame', 'map' )

        with open('test.csv', 'r') as f:
            reader = csv.reader(f)

            for cols in reader:
                goalList.append([float(value) for value in cols])

            goalList = np.array(goalList)
            print("read suc!!")
            goalListX = goalList[:,0]
            goalListY = goalList[:,1]
            goalListZ = goalList[:,2]
            goalListW = goalList[:,3]


        if len(goalListX) == len(goalListY) & len(goalListY) >= 1:
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalListX, goalListY, retry, map_frame)
            rospy.spin()
        else:
            rospy.loginfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")


