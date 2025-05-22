#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>

// 批量停止多个节点
void stopNodes(const std::vector<std::string>& node_names) {
    if (node_names.empty()) return;

    // 构建 ros2 node kill 命令一次性停止多个节点
    for (const auto& node_name : node_names) {
        std::string command = "ros2 lifecycle set " + node_name + " shutdown";
        int ret = system(command.c_str());
        if (ret != 0) {
            std::cerr << "Failed to kill node " << node_name << std::endl;
        }
        else {
            std::cout << "Node " << node_name << " killed successfully." << std::endl;
        }
    }
}

// 异步启动 ROS 2 启动文件（避免阻塞）
void startLaunchFileAsync(const std::string& launch_file) {
    std::thread([launch_file]() {
        std::string command = "ros2 launch " + launch_file;
        int ret = system(command.c_str()); // 使用 system 命令启动
        if (ret != 0) {
            std::cerr << "Failed to start launch file " << launch_file << std::endl;
        }
        else {
            std::cout << "Launch file " << launch_file << " started successfully." << std::endl;
        }
        }).detach();  // 异步执行，主线程不会阻塞
}

int main() {
    // 要停止的节点列表
    std::vector<std::string> nodes_to_kill = { "laser_go", "my_car_control", "wheel_odom", "ekf_se" };

    // 停止节点（通过系统命令）
    stopNodes(nodes_to_kill);

    // 异步启动 launch 文件
    std::string launch_file = "/home/racecar/桌面/hello_ws/src/racecar/launch/my_launch/auto_nav_try.launch";
    startLaunchFileAsync(launch_file);

    // 程序等待3秒，确保所有操作在3秒内完成
    std::this_thread::sleep_for(std::chrono::seconds(3));

    return 0;
}
