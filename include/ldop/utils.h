#pragma once

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

namespace ldopcore {

// 统一把 steady_clock 的时间差转换成毫秒，避免各模块重复写样板代码。
inline double elapsedMs(const std::chrono::steady_clock::time_point& start,
                        const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

// 统一毫秒日志输出精度，保证不同模块的 timing 日志格式一致。
inline std::string formatFloatMs(const double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

}  // namespace ldopcore
