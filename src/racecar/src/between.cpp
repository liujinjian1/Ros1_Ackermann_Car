#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h" // 里程计消息
#include <vector>
#include <math.h>
#include <iostream>
#include "geometry_msgs/Pose.h"
#include <fstream>
#include <cmath>
#include <tf/tf.h>
#define PI 3.1415926

float flat_x = 0;
float flat_y = 0;

typedef struct // 锥桶信息结构体
{
    float angle;
    float distance;
    float dk_x;
    float dk_y;
} dir_obstacle;

class class_laser_2d
{
private:
    int ind_left[360] = { 0 };         // 排序后元素对应的位置
    float new_laser_left[360] = { 0 }; // 拼接后的数据
    int ind_right[360] = { 0 };        // 排序后元素对应的位置
    float new_laser_right[360] = { 0 }; // 拼接后的数据
    int ind_yang[360] = { 0 };
    float new_laser_yang[360] = { 0 };
    int ind_yi[360] = { 0 };
    float new_laser_yi[360] = { 0 };

    ros::NodeHandle nh;
    geometry_msgs::Pose point; // 目标中点坐标
    ros::Subscriber sub;
    ros::Subscriber odom_sub; // 里程计订阅者
    ros::Publisher pub;
    dir_obstacle ob_1, ob_2, ob_3, ob_4;

    std::ofstream output_file; // 文件流
    std::ofstream yaw_output_file; // 新增文件流，用于目标航向

    void Get_goal(const sensor_msgs::LaserScan::ConstPtr& msg_p);
    //void OdometryCB(const nav_msgs::Odometry::ConstPtr& msg); // 里程计回调函数
    void BubbleSort(float* p, int length, int* ind_diff);
    void writeToCSV(float x, float y, float z, float w);
    void writeYawToCSV(float current_yaw_, float x, float y); // 修改写入目标航向的函数

    std::pair<float, float> last_position = { 0.0, 0.0 }; // 上一个位置
    bool initialized = false; // 是否已初始化
    bool first_write = true; // 标记第一次写入
    int current_path_index = 0; // 当前路径索引

public:
    class_laser_2d();
    ~class_laser_2d();
    float last_distance = 0;
    float accDistance = 0;
    float flogGO = 0;
    float flogStop = 0;
    float lastZ = 0;
    float x = 0;
    float y = 0;
    double current_yaw_ = 0;
    float flogSart = 0; 
};

class_laser_2d::class_laser_2d()
{
    ROS_INFO("雷达数据分析节点开启");
    sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_1", 100, &class_laser_2d::Get_goal, this);
    //odom_sub = nh.subscribe<nav_msgs::Odometry>("/encoder_imu_odom", 100, &class_laser_2d::OdometryCB, this);
    pub = nh.advertise<geometry_msgs::Pose>("/point_mid", 100, true);

    // 清空文件并写入标题
    //std::string filename = "/home/racecar/桌面/hello_ws/src/racecar/scripts/text.csv";
    //output_file.open(filename, std::ios::trunc); // 以截断模式打开，清空文件

    // 新增目标航向文件流
    std::string yaw_filename = "/home/racecar/桌面/hello_ws/src/racecar/scripts/yaw_data.csv";
    yaw_output_file.open(yaw_filename, std::ios::trunc); // 清空文件
    yaw_output_file << "target_yaw,x,y\n"; // 写入标题
}

class_laser_2d::~class_laser_2d()
{
    printf("雷达数据分析节点关闭\n");
    output_file.close(); // 关闭文件
    yaw_output_file.close(); // 关闭目标航向文件
}

void class_laser_2d::writeToCSV(float x, float y, float z, float w)
{
   // output_file << x << "," << y << "," << z << "," << w << "\n"; // 写入数据
}

void class_laser_2d::writeYawToCSV(float current_yaw_, float x, float y)
{
    yaw_output_file << current_yaw_ << "," << x << "," << y << "\n"; // 写入目标航向及位置数据
}

void class_laser_2d::Get_goal(const sensor_msgs::LaserScan::ConstPtr& msg_p)
{
    int num_left = 0;
    int num_right = 0;
    int num_yang = 0;
    int num_yi = 0;

    // 进行数据拼接
    for (int i = 1080; i <= 1440; i++)
    {
        new_laser_right[num_right] = msg_p->ranges[i];
        num_right++;
    }
    for (int i = 0; i <= 360; i++)
    {
        new_laser_left[num_left] = msg_p->ranges[i];
        num_left++;
    }
    for (int i = 360; i <= 720; i++)
    {
        new_laser_yang[num_yang] = msg_p->ranges[i];
        num_yang++;
    }
    for (int i = 720; i <= 1080; i++)
    {
        new_laser_yi[num_yi] = msg_p->ranges[i];
        num_yi++;
    }

    BubbleSort(new_laser_right, 360, ind_right);
    BubbleSort(new_laser_left, 360, ind_left);
    BubbleSort(new_laser_yang, 360, ind_yang);
    BubbleSort(new_laser_yi, 360, ind_yi);

    ob_1.angle = (ind_left[0] + 0) / 4;
    ob_1.distance = new_laser_left[0];
    ob_2.angle = (ind_right[0] + 1080) / 4;
    ob_2.distance = new_laser_right[0];
    ob_3.angle = (ind_yang[0] + 360) / 4;
    ob_3.distance = new_laser_yang[0];
    ob_4.angle = (ind_yi[0] + 720) / 4;
    ob_4.distance = new_laser_yi[0];

    ob_1.dk_x = -abs(ob_1.distance * cos(ob_1.angle * PI / 180)); // 二向县
    ob_1.dk_y = abs(ob_1.distance * sin(ob_1.angle * PI / 180));
    ob_2.dk_x = abs(ob_2.distance * cos(ob_2.angle * PI / 180));  // 一向县
    ob_2.dk_y = abs(ob_2.distance * sin(ob_2.angle * PI / 180));
    ob_3.dk_x = -abs(ob_3.distance * cos(ob_3.angle * PI / 180)); // 三向县
    ob_3.dk_y = -abs(ob_3.distance * sin(ob_3.angle * PI / 180));
    ob_4.dk_x = abs(ob_4.distance * cos(ob_4.angle * PI / 180));  // 四向县
    ob_4.dk_y = -abs(ob_4.distance * sin(ob_4.angle * PI / 180));


    // 在目标航向计算后写入 CSV


    if (std::isinf(ob_3.distance) || std::isinf(ob_4.distance)) 
    {
        if(std::isinf(ob_3.distance))
        {
            point.position.x = (ob_1.dk_x + ob_4.dk_x) / 2;
            point.position.y = (ob_1.dk_y + ob_4.dk_y) / 2;
            point.orientation.z = 1;
            point.orientation.w = 1;
            pub.publish(point);
        }
        else
        {
            point.position.x = (ob_2.dk_x + ob_3.dk_x) / 2;
            point.position.y = (ob_2.dk_y + ob_3.dk_y) / 2;
            point.orientation.z = 1;
            point.orientation.w = 1;
            pub.publish(point);
        }
    }
    else if (!(std::isinf(ob_3.distance) || std::isinf(ob_4.distance)))
    {
        if(ob_1.distance < ob_2.distance)
        {
            point.position.x = (ob_1.dk_x + ob_4.dk_x) / 2;
            point.position.y = (ob_1.dk_y + ob_4.dk_y) / 2;
            point.orientation.z = 1;
            point.orientation.w = 1;
            pub.publish(point);
           // ROS_INFO("ob_1.dk_x :%f",ob_1.dk_x);
            //    ROS_INFO("ob_1.dk_y :%f",ob_1.dk_y);
              //  ROS_INFO("ob_4.dk_x :%f",ob_4.dk_x);
             //   ROS_INFO("ob_4.dk_y :%f",ob_4.dk_y);
        }
        else if (ob_1.distance > ob_2.distance)
        {
            point.position.x = (ob_2.dk_x + ob_3.dk_x) / 2;
            point.position.y = (ob_2.dk_y + ob_3.dk_y) / 2;
            point.orientation.z = 1;
            point.orientation.w = 1;
            pub.publish(point);
         //   ROS_INFO("ob_2.dk_x :%f",ob_2.dk_x);
          //      ROS_INFO("ob_2.dk_y :%f",ob_2.dk_y);
           //     ROS_INFO("ob_3.dk_x :%f",ob_3.dk_x);
             //   ROS_INFO("ob_3.dk_y :%f",ob_3.dk_y);
        }
    }

   
}

/*void class_laser_2d::OdometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{   
    // ROS_INFO("---------------------------------\n");  
     x = msg->pose.pose.position.x;
     y = msg->pose.pose.position.y;
    float z = msg->pose.pose.orientation.z;
    float w = msg->pose.pose.orientation.w;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    if (!initialized)
    {
        last_position = { x, y };
        initialized = true;
    }
    else
    {
   
        
        float distance = sqrt(pow(x - last_position.first, 2) + pow(y - last_position.second, 2));
        if (distance >= 0.1)
        {
           //ROS_INFO("------------------x----------------:%f\n",x);
          // ROS_INFO("------------------y----------------:%f\n",y);
          // ROS_INFO("------------------current_yaw_----------------:%f\n",current_yaw_);
     
           // writeYawToCSV(current_yaw_, x, y); // 将目标航向及位置写入 CSV
            last_position = { x, y };        
            accDistance += 0.1;               // 累加距离
            ROS_INFO("accDistance:%f", accDistance);  
        }
        
        if (accDistance > 7.2 && accDistance < 12.5)
        {  
            if ((accDistance - last_distance) >= 0.3)
            {   
             //writeToCSV(x, y, w, z); // 将目标航向及位置写入 CSV 
             ROS_INFO("写入成功");      
             last_distance = accDistance;
            }
        }
        
        if ( flogSart == 0 && accDistance < 1.2  ){
          // writeToCSV(2, 0, w, z);  
           ROS_INFO("第一点写入成功");  
          // writeToCSV(4, 0, w, z);  
           ROS_INFO("第二点写入成功");   
           flogSart = 1;   
            
                     
        }
        

        if (accDistance > 12.5)
        {
            if ((accDistance - last_distance) >= 4.0)
            {
                flat_x = point.position.x + x;
                flat_y = point.position.y + y;
                
                
              //  ROS_INFO("flat_x:%f",flat_x);
             //   ROS_INFO("flat_y:%f",flat_y);
             //   ROS_INFO("point.position.x:%f",point.position.x);
            //     ROS_INFO("point.position.y:%f",point.position.y);
                ROS_INFO("写入成功");
                //writeToCSV(x, y, z, w);
                last_distance = accDistance; // 更新位置
            }
        }

        if (accDistance > 3.0 && flogGO == 0)
        {
            flogGO = 1;
        }
        if ((sqrt(pow(x, 2) + pow(y, 2))) < 1 && flogGO == 1 && flogStop == 0)
        {
           // writeToCSV(0, 0, w, z);  
            ROS_INFO("最后点写入成功");  
            flogStop = 1;
        }  
    }
}
*/
void class_laser_2d::BubbleSort(float* p, int length, int* ind_diff)
{
    for (int m = 0; m < length; m++)
    {
        ind_diff[m] = m;
    }

    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length - i - 1; j++)
        {
            if (p[j] > p[j + 1])
            {
                float temp = p[j];
                p[j] = p[j + 1];
                p[j + 1] = temp;

                int ind_temp = ind_diff[j];
                ind_diff[j] = ind_diff[j + 1];
                ind_diff[j + 1] = ind_temp;
            }
        }
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "laser_point_pub");

    class_laser_2d laser_point_pub;

    ros::spin();
    return 0;
}
