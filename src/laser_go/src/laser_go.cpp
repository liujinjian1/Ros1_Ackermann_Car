#include <string>
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "Control.h"

#define pi_2 1.57
double last_angle = 90;
int index_angle = 0;
int zero_count = 0;
double initial_x = 0.0;
bool start_turn = false;
bool turn_completed = false;
bool return_to_x = false;

const int freq = 1440;
const int FREQ = 1440; // 每转一圈雷达扫描次数
const double VMAX = 1980.0; // 最大速度19100
const double MIN_RANGE = 0.0; // 最小有效距离
const double MAX_RANGE = 2.5; // 最大有效距离
const double SECTOR_SIZE = 1; // 每个扇区的大小（度数）
const double THRESHOLD_Y = 2.3; // 阈值   2.1
const double q = -1.2; // 常数
const double w1 = 0.4; // 常数
const double w2 = 10.0; // 常数
const double DEFAULT_RANGE = 5.0; // 处理错误距离
double rate_p[10] = { 0.8, 2.2, 0.8, 0, 0, 0, 0, 0, 0, 0 };
double max_right_dis = 1.5;
double max_left_dis = 1.5;
int middle_angle = 90;
int bucket_threshold = 5;
int vaild_bucket_threshold = 2;

typedef struct
{
    double range;
    double theta;
} Point_polar;

typedef struct
{
    double x;
    double y;
} Point_rectangular;

Control::PID pid;

void cal_point(double theta, double range, double& tmp_x, double& tmp_y)
{
    tmp_x = range * sin(theta);
    tmp_y = range * cos(theta);
}

class PubAndSub : public rclcpp::Node
{
public:
    PubAndSub() : Node("laser_go")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/car/cmd_vel", 5);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 15, std::bind(&PubAndSub::callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PubAndSub::odomCallback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::LaserScan::SharedPtr laser);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

void PubAndSub::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

}

void PubAndSub::callback(const sensor_msgs::msg::LaserScan::SharedPtr laser)
{
    auto twist = geometry_msgs::msg::Twist();
     //if (!turn_completed) return;
    bool is_turn_left = false; // false 表示向右转，true 表示向左转
    const double adjustment_threshold = 20.0; // 允许的微调角度范围（度） 
    int num_sectors = 180; // 360度划分为2.5度的扇区
    std::vector<double> dimin(num_sectors, std::numeric_limits<double>::max());
    // 遍历激光雷达数据，只取第一和第四象限的点
    for (int i = 0; i < FREQ; i++) {
        if (laser->ranges[i] > MIN_RANGE && laser->ranges[i] < MAX_RANGE && abs(laser->ranges[i] - laser->ranges[i-1])>1.0 ) {
            double theta = i * laser->angle_increment + laser->angle_min;
            if ((theta >= 0 && theta <= M_PI / 2) || (theta >= 3 * M_PI / 2 && theta <= 2 * M_PI)) {
                int sector_index = 0;
                if (theta >= 0 && theta <= M_PI / 2) {
                  sector_index = 90+static_cast<int>((theta * 180 / M_PI) / SECTOR_SIZE); // 第一象限
                } else {
                   sector_index = 90-static_cast<int>(abs(theta - 2 * M_PI ) * 180 / M_PI / SECTOR_SIZE) ; // 第四象限
                }
                if (sector_index >= 0 && sector_index < num_sectors) {
                    double range_value = static_cast<double>(laser->ranges[i]);
                    dimin[sector_index] = std::min(dimin[sector_index], range_value);
                }
            }
        }
    }

    // 筛选出可行区域并合并相邻区域
    std::vector<int> candidates;
    for (int i = 0; i < num_sectors; i++) {
        if (dimin[i] >= THRESHOLD_Y) {
            candidates.push_back(i);
        }
    }


    double best_direction = 0.0;
    double max_range = 0.0;
    int max_length = 0;
    double dmin = std::numeric_limits<double>::max();

    for (size_t i = 0; i < candidates.size(); i++) {
        int start = candidates[i];
        while (i < candidates.size() - 1 && candidates[i + 1] == candidates[i] + 1) {
            i++;
        }
        int end = candidates[i];
        //ROS_INFO("start: %d ,end: %d", start, end);

        // 计算这个可行区域的中心方向

        int length = end - start;
       //ROS_INFO("length: %d ",  length);
        double direction = ((start + end) / 2.0) * SECTOR_SIZE * M_PI / 180.0;
        double yang = direction*180/3.1415926 ;
        double range = DEFAULT_RANGE;
        for(int j =start;j<=end;j++)
        {
            if(dimin[j]!=std::numeric_limits<double>::max()){
            range = std::min(range,dimin[j]);
            }
        }
        
        if (length > max_length )
         {
            max_length = length;
            max_range = range;
            best_direction = direction;
            dmin = range;
        }
        
    }   
    

    double best_direction_1 = pid.PIDPositional(best_direction);
    if (best_direction_1 > 180)
        best_direction_1 = 180;
    if (best_direction_1 < 0)
        best_direction_1 = 0;
    double a =abs(90-best_direction_1); // 转弯角度
    double v = w1 * a + w2 * dmin; // 计算速度
    double speed = VMAX + q * v ; // 最终速度

    twist.linear.x = speed;
    twist.angular.z = best_direction_1;

    //should_turn = 0;
    pub_->publish(twist);
}

int main(int argc, char* argv[])
{
    pid.Init();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubAndSub>());
    rclcpp::shutdown();
    return 0;
}
