#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

// 相机模型的库，这些头文件涉及不同的相机模型，例如针孔相机、鱼眼相机等
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

// 
#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

// inBorder 函数声明，用于判断一个点是否在图像的边界内。
bool inBorder(const cv::Point2f &pt);

// reduceVector 函数声明，用于根据状态向量 status 从输入的向量 v 中去除不需要的元素。
// 重载·？
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    // readImage 函数，用于读取当前图像和时间戳。
	void readImage(const cv::Mat &_img,double _cur_time);
	// setMask 函数，设置图像的掩码（可能用于特定区域的处理）。
    void setMask();
	// addPoints 函数，用于添加新的特征点。
    void addPoints();
	// updateID 函数，用于更新特征点的 ID。
    bool updateID(unsigned int i);
	// readIntrinsicParameter 函数，用于读取相机的内参，通常包括焦距、主点坐标等信息。
    void readIntrinsicParameter(const string &calib_file);
	// showUndistortion 函数，用于展示去畸变后的图像。
    void showUndistortion(const string &name);
	// rejectWithF 函数，可能用于通过基础矩阵（F）来剔除一些错误匹配的点对。
	// ？
    void rejectWithF();
	// undistortedPoints 函数，去除畸变，得到未畸变的特征点。
    void undistortedPoints();
	
	// mask：图像的掩码，指定图像处理的区域。
	// fisheye_mask：鱼眼图像的掩码
    cv::Mat mask;
    cv::Mat fisheye_mask;
	// 前一帧、当前帧和下一帧的图像。
    cv::Mat prev_img, cur_img, forw_img;
	// n_pts：存储新的特征点。
	// prev_pts、cur_pts 和 forw_pts：分别存储前一帧、当前帧和下一帧的特征点。
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
	// prev_un_pts 和 cur_un_pts：分别存储前一帧和当前帧去畸变后的特征点。
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
	// pts_velocity：存储特征点的速度（即特征点的运动信息）。
    vector<cv::Point2f> pts_velocity;
	// ids：存储特征点的 ID。
    vector<int> ids;
	// track_cnt：存储特征点的跟踪计数，用于判断特征点是否稳定。
    vector<int> track_cnt;
	// cur_un_pts_map 和 prev_un_pts_map：分别存储当前帧和前一帧去畸变后的特征点与其 ID 的映射。
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
	// m_camera：相机对象，用于获取和处理相机参数。
    camodocal::CameraPtr m_camera;
	// cur_time 和 prev_time：分别表示当前帧和前一帧的时间戳。
    double cur_time;
    double prev_time;
	// n_id：静态成员变量，用于生成唯一的特征点 ID。
    static int n_id;
};
