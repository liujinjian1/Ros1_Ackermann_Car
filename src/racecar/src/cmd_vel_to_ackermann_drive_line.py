#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: christoph.roesmann@tu-dortmund.de
import rospy, math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import PID_Control
import time
car_speed_out = 1500.0
stop_flag = 0
stop_start_time = 0
stop_duration = 2  # 停车时间为3秒
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def cmd_callback(data):
    global speed_control
    global angle_control
    global car_v_callback
    global speed_pid_fhspeed
    global car_speed_out
    global stop_flag
    global stop_start_time

    if stop_flag == 1:
        # 检查停车时间是否已经超过3秒
        if time.time() - stop_start_time >= stop_duration:
            stop_flag = 0  # 重置标志位，继续导航
        else:
            # 发布停止命令
            msg = Twist()
            msg.linear.x = 1500
            msg.angular.z = 90
            pub.publish(msg)
            return




    speed_control = data.linear.x
    angle_control = data.angular.z

    msg = Twist()  

    cmd_front_max = 1533
    cmd_front_min = 1500
    cmd_back_min = 1480
    cmd_back_max = 1250

    car_speed_error = speed_control - car_v_callback

    if speed_control > 0.1 and speed_control <26:
        msg.linear.x = 3.006*speed_control**3-24.44*speed_control**2+79.28*speed_control+1510

    if msg.linear.x > cmd_front_max:
        msg.linear.x = cmd_front_max
    if stop_flag == 2:
        msg = Twist()
        msg.linear.x = 1529

    # msg.z是经函数处理后最终发布的角度数据
    msg.angular.z = convert_trans_rot_vel_to_steering_angle(speed_control, angle_control, 0.35)*180/3.1416+90
    #print("Control speed is {:5.2f}, output is {:5.2f}, Control angle is {:5.2f}, output is {:5.2f}".format(speed_control, msg.linear.x, angle_control*(180.0/3.1416),msg.angular.z))
    pub.publish(msg)

def arrfinal_callback(data):
    rospy.loginfo("Received data: %f", data.data)
    global stop_flag
    global stop_start_time
    
    stop_flag = data.data
    stop_start_time = time.time()


def callback_read_current_position(data):
    global car_v_callback
    car_v_callback = data.twist.twist.linear.x


if __name__ == '__main__': 
  try:
    rospy.init_node('cmd_vel_to_ackermann_drive')
        

    speed_pid_fhspeed = PID_Control.PID_Control()

    speed_pid_fhspeed.Kp = 5.0
    speed_pid_fhspeed.Ki = 3.0
    speed_pid_fhspeed.Kd = 0.0

    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    artcar_cmd_topic = rospy.get_param('~artcar_cmd_topic', '/car/cmd_vel')
    wheelbase = rospy.get_param('~wheelbase', 0.35)
    frame_id = rospy.get_param('~frame_id', 'odom')
    car_odometry_get = rospy.get_param('~car_odometry_get', '/odometry/filtered')

    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    rospy.Subscriber(car_odometry_get, Odometry, callback_read_current_position, queue_size=1)
    rospy.Subscriber('/arrfinal', Float64, arrfinal_callback)
    pub = rospy.Publisher(artcar_cmd_topic, Twist, queue_size=1)
    
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", artcar_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    twist = Twist()
    twist.linear.x = 1500; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 90
    pub.publish(twist)
    pass

  finally:
    twist = Twist()
    twist.linear.x = 1500; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 90
    pub.publish(twist)

