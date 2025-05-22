#define _pose sqrt(pow(carpose_odom.pose.position.x, 2) + pow(carpose_odom.pose.position.y, 2)) //D?3¦Ì?¨¤¨¤??e¦Ì??¨¤¨¤?
#define _pose_red sqrt(pow(carpose_map.pose.position.x+3.5, 2) + pow((carpose_map.pose.position.y), 2)) //D?3¦Ì?¨¤¨¤?red?¨¤¨¤?
#define _pose_stop sqrt(pow(carpose_odom.pose.position.x, 2) + pow((carpose_odom.pose.position.y), 2)) //D?3¦Ì?¨¤¨¤?¨ª¡ê3¦Ì???¨¤¨¤?
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //¡Á¡é¨°a: ¦Ì¡Â¨®? transform ¡À?D?¡ã¨¹o???¨ª¡¤???t
#include <vector>
#include <cmath>
#include <iostream>
#include "dynamic_reconfigure/server.h"
#include "racecar/drConfig.h"
#include <unistd.h>
#include <sys/types.h>

using std::vector;
typedef struct //¡¤??¨°PID?¨¢11¨¬?
{
    float error_angle;
    float error_line;
    float P_angle;
    float D_angle;
    float P_line;
    float D_line;
    float last_error_angle;
    float last_error_line;
    int out_angle;
    int out_line;
    float line_D;
    float angle_D;
} pid_dir;

pid_dir PID_dir;

class my_car_control
{
private:
    bool flag_go = 0,    //????¨¤??a?-¦Ì?¡À¨º????
        flag_red = 0,
        flag_red_first = 0,
        flag_finish = 0,
        flag_stop = 0,
        flag_stop_last =0; //????¦Ì¨²¨°?¨¨|¨ª¨º3¨¦¡À¨º????
    ros::NodeHandle n_;
    ros::Subscriber laser_sub;
    ros::Subscriber go_sub;
    ros::Subscriber carpose_sub;
    ros::Publisher vel_pub;

    tf2_ros::Buffer buffer;
    tf2::Quaternion qtn;
    geometry_msgs::Twist cmd_vel;                  //D?3¦Ì?¨´?¨¨???¨®
    geometry_msgs::PoseStamped point_baselink_pub; //¡¤¡é2???¡À¨º?D¦Ì?¡Á?¡À¨º  frame_id:baselink
    geometry_msgs::PoseStamped point_map_pub;      //¡¤¡é2???¡À¨º?D¦Ì?¡Á?¡À¨º  frame_id:map
    geometry_msgs::PoseStamped carpose_odom;       //D?3¦Ì????¡Á?¡À¨º    frame_id:map
    geometry_msgs::PoseStamped carpose_map;        //D?3¦Ì????¡Á?¡À¨º    frame_id:map
    vector<geometry_msgs::PoseStamped> vec_pub;    //¡ä?¡ä¡é?¨´¨®D¡¤¡é2???¡À¨º?D¦Ì?¦Ì?¨¨Y?¡Â
    void car_pose(const nav_msgs::Odometry::ConstPtr &odomMsg);
    void laser_callback(const geometry_msgs::Twist::ConstPtr &cmd_velMsg);
    void go_callback(const geometry_msgs::Twist::ConstPtr &cmd_velMsg);

public:
    my_car_control();
    ~my_car_control();
    float dis_point(const geometry_msgs::PoseStamped &x1, const geometry_msgs::PoseStamped &x2);
};

my_car_control::my_car_control()
{
   

    tf2_ros::TransformListener listener(buffer);
    ros::Rate r(1);
    r.sleep();
    /* ????/¡¤¡é2?3?¨º??¡¥ */
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;

    point_map_pub.pose.position.z = 0;
    point_baselink_pub.header.frame_id = "base_link";
    carpose_odom.header.frame_id = "odom";

    cmd_vel.linear.x = 0;
    laser_sub =  n_.subscribe<geometry_msgs::Twist>("/car/cmd_vel", 10, &my_car_control::laser_callback, this);
    go_sub =  n_.subscribe<geometry_msgs::Twist>("/go/cmd_vel", 10, &my_car_control::go_callback, this);
    carpose_sub = n_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, &my_car_control::car_pose, this);
    vel_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 100); 
}

my_car_control::~my_car_control()
{
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = 90;
    vel_pub.publish(cmd_vel);
    
    printf("-------------------------\n");
}


void my_car_control::car_pose(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    try
    {
      
        carpose_odom.pose.position.x = odomMsg->pose.pose.position.x;
        carpose_odom.pose.position.y = odomMsg->pose.pose.position.y;
        carpose_odom.pose.position.z = odomMsg->pose.pose.position.z;
        carpose_odom.pose.orientation.x = odomMsg->pose.pose.orientation.x;
        carpose_odom.pose.orientation.y = odomMsg->pose.pose.orientation.y;
        carpose_odom.pose.orientation.z = odomMsg->pose.pose.orientation.z;
        carpose_odom.pose.orientation.w = odomMsg->pose.pose.orientation.w;
        carpose_map = buffer.transform(carpose_odom, "map");
        ROS_INFO(" carpose_odom.pose.position.x:%f", _pose_stop);
     
    }
    
    catch (const std::exception &e)
    {
       
       
    }
    
    ROS_INFO("----------------------_pose_stop:%f------------------",_pose_stop);
    if (_pose > 3 && flag_go == 0) 
    {
        flag_go = 1;
       
    }
    
  if ((_pose_stop <= 0.95) && (flag_finish == 1)&& (flag_go==1)&&flag_stop_last!=1 && flag_stop==0) 
    {
        ROS_INFO("----------------开始停车1--------------------");
      cmd_vel.linear.x = 775;
        cmd_vel.angular.z = 90;
        vel_pub.publish(cmd_vel);
        vel_pub.publish(cmd_vel);
       // ros::Duration(1.2).sleep();
        vel_pub.publish(cmd_vel);
        vel_pub.publish(cmd_vel);
        vel_pub.publish(cmd_vel);
        cmd_vel.linear.x = 1500;
        cmd_vel.angular.z = 90;
        vel_pub.publish(cmd_vel);
        ros::Duration(3).sleep();      
        flag_stop = 1;             
    }

  if(_pose_stop > 2.5 && flag_stop==1 && flag_stop_last!=1)
   {
        ROS_INFO("--------turn left--------\n");
        cmd_vel.linear.x = 1800;
        cmd_vel.angular.z = 130.0; 
        vel_pub.publish(cmd_vel);
        ros::Duration(0.3).sleep();
        ROS_INFO("--------turn right--------\n");
        cmd_vel.linear.x = 1800;
        cmd_vel.angular.z = 30.0; 
        vel_pub.publish(cmd_vel);
        ros::Duration(0.35).sleep();
        flag_stop_last = 1;
        cmd_vel.linear.x = 1500;
        cmd_vel.angular.z = 90;
        vel_pub.publish(cmd_vel);
        ros::Duration(300).sleep();   
        
   }
   
   
/*   
   if(_pose_stop > 3.5 && flag_stop_last ==1)
   {
  	ROS_INFO("----------------开始停车2--------------------");
          cmd_vel.linear.x = 775;
        cmd_vel.angular.z = 90;
        vel_pub.publish(cmd_vel);
        vel_pub.publish(cmd_vel);
       // ros::Duration(1.2).sleep();
        vel_pub.publish(cmd_vel);
        vel_pub.publish(cmd_vel);
        vel_pub.publish(cmd_vel);
        cmd_vel.linear.x = 1500;
        cmd_vel.angular.z = 90;
        vel_pub.publish(cmd_vel);
        ros::Duration(300).sleep();   
   }
   */
    if (_pose<1.2 && flag_go==1) 
    {
        flag_finish = 1; 
        flag_go =1;
    }
}
void my_car_control::laser_callback(const geometry_msgs::Twist::ConstPtr &cmd_velMsg)
{
    if(!flag_red)
    {
        
        cmd_vel.linear.x = cmd_velMsg->linear.x;
        cmd_vel.angular.z = cmd_velMsg->angular.z;
    }
    else
    {
        ros::Duration(0.7).sleep();
        flag_red=0;
    }
    vel_pub.publish(cmd_vel);
}

void my_car_control::go_callback(const geometry_msgs::Twist::ConstPtr &cmd_velMsg)
{
   
}   

float my_car_control::dis_point(const geometry_msgs::PoseStamped &x1, const geometry_msgs::PoseStamped &x2)
{
    float dis;
    dis = sqrt(pow(x1.pose.position.x - x2.pose.position.x, 2) + pow(x1.pose.position.y - x2.pose.position.y, 2));
   
    return dis;
    
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "my_car_control");
    my_car_control controller;

    ros::spin();
    return 0;
}


