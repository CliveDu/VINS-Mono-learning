#pragma once

#include <ctime>
#include <cstdlib>
// 提供了计时功能，能够处理时间点、时间段等操作。可用于计算时间差
#include <chrono>

// 
class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    // 被构造函数调用，实例化时直接开始计时
	void tic()
    {
        // 使用 std::chrono::system_clock::now() 获取当前时间点作为计时起点
		start = std::chrono::system_clock::now();
    }

    // 返回从调用 tic() 到调用 toc() 之间的时间差
	double toc()
    {
        end = std::chrono::system_clock::now();
		// 计算时间差，返回的是一个 std::chrono::duration 类型的对象，表示两个时间点之间的间隔。
        std::chrono::duration<double> elapsed_seconds = end - start;
		// 取 elapsed_seconds 的数值部分，并将其乘以 1000 转换为毫秒（count() 返回的默认单位是秒，因此乘以 1000 进行毫秒转换）。
        return elapsed_seconds.count() * 1000;
    }

  private:
    // start 和 end 是 std::chrono::time_point 类型的变量
	std::chrono::time_point<std::chrono::system_clock> start, end;
};
