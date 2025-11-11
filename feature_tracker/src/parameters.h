// 编译指令，确保头文件只被编译一次，防止多次包含。
#pragma once
// 包含 ROS 的头文件，用于在 ROS 环境中创建节点、发布话题、订阅话题等。
#include <ros/ros.h>
// OpenCV 高级图像处理头文件，用于处理图像窗口和图形界面相关操作（例如显示图像）。
#include <opencv2/highgui/highgui.hpp>

// extern int ROW;：外部声明一个整型变量 ROW，通常用于存储图像的行数（即图像的高度）。
// extern int COL;：外部声明一个整型变量 COL，通常用于存储图像的列数（即图像的宽度）。
// extern int FOCAL_LENGTH;：外部声明一个整型变量 FOCAL_LENGTH，通常表示相机的焦距。
// const int NUM_OF_CAM = 1;：常量 NUM_OF_CAM 用于表示相机的数量，这里设置为 1，意味着使用单目相机。
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


// extern std::string IMAGE_TOPIC;：外部声明一个字符串 IMAGE_TOPIC，表示图像数据的 ROS 话题名。
// extern std::string IMU_TOPIC;：外部声明一个字符串 IMU_TOPIC，表示惯性测量单元（IMU）数据的 ROS 话题名。
// extern std::string FISHEYE_MASK;：外部声明一个字符串 FISHEYE_MASK，可能用于存储鱼眼镜头的掩膜路径（如果使用鱼眼相机）。
// extern std::vector<std::string> CAM_NAMES;：外部声明一个字符串向量 CAM_NAMES，用于存储相机的名称。
extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
// extern int MAX_CNT;：外部声明一个整数变量 MAX_CNT，表示最大特征点数目。
// extern int MIN_DIST;：外部声明一个整数变量 MIN_DIST，表示最小特征点间的距离，通常用于特征点的去重复操作。
// extern int WINDOW_SIZE;：外部声明一个整数变量 WINDOW_SIZE，通常表示图像滑动窗口的大小。
// extern int FREQ;：外部声明一个整数变量 FREQ，通常表示系统运行频率或图像帧率。
// extern double F_THRESHOLD;：外部声明一个双精度浮动变量 F_THRESHOLD，可能用于特征匹配时的阈值判断。
// extern int SHOW_TRACK;：外部声明一个整数变量 SHOW_TRACK，表示是否显示特征点跟踪。
// extern int STEREO_TRACK;：外部声明一个整数变量 STEREO_TRACK，可能用于立体相机设置，表示是否启用立体跟踪。
// extern int EQUALIZE;：外部声明一个整数变量 EQUALIZE，表示是否执行直方图均衡化（通常用于增强图像对比度）。
// extern int FISHEYE;：外部声明一个整数变量 FISHEYE，可能用于指示是否使用鱼眼相机。
// extern bool PUB_THIS_FRAME;：外部声明一个布尔变量 PUB_THIS_FRAME，表示是否发布当前帧数据。
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
// readParameters(ros::NodeHandle &n)：该函数用于读取 ROS 参数服务器上的参数并初始化相关变量。
// ros::NodeHandle 是 ROS 中用于管理节点相关操作的类，n 是节点句柄，函数的具体实现可能会根据 ROS 参数加载配置文件中的参数值。
void readParameters(ros::NodeHandle &n);
