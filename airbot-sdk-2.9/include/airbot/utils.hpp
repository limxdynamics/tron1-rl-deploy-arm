#ifndef UTILS_HPP
#define UTILS_HPP

#include <math.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>

#include "airbot/command/command_types.hpp"

inline std::string get_timestring() {
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
  return ss.str();
}

inline uint64_t get_millisecond_now() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}
inline uint64_t get_microsecond_now() {
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

template <typename T>
constexpr inline T limit(T val, const T& min, const T& max) {
  return std::clamp(val, min, max);
}

template <typename T>
constexpr inline T limit(T val, const T& limit) {
  T max = limit < 0 ? -limit : limit;
  T min = limit > 0 ? -limit : limit;
  return std::clamp(val, min, max);
}

template <typename T>
constexpr inline T limit_max(T val, const T& max) {
  return std::clamp(val, std::numeric_limits<T>::min(), max);
}

template <typename T>
constexpr inline T limit_min(T val, const T& min) {
  return std::clamp(val, min, std::numeric_limits<T>::max());
}

template <typename T, typename U>
constexpr inline U range_map(T val, T in_min, T in_max, U out_min, U out_max) {
  return static_cast<U>(static_cast<double>((val - in_min)) * (out_max - out_min) / (in_max - in_min) + out_min);
}

template <typename T>
constexpr inline T bitmax(uint64_t n) {
  static_assert(std::is_integral<T>::value, "T must be an integral type");
  return static_cast<T>((1ULL << n) - 1);
}

template <typename T>
inline double l1norm(const T& a, const T& b) {
  assert(a.size() == b.size());
  double sum = 0;
  for (size_t i = 0; i < a.size(); i++) sum += fabs(a[i] - b[i]);
  return sum;
}

template <typename T>
inline double l2norm(const T& a, const T& b) {
  assert(a.size() == b.size());
  double sum = 0;
  for (size_t i = 0; i < a.size(); i++) sum += (a[i] - b[i]) * (a[i] - b[i]);
  return sqrt(sum);
}

inline double diff(const Frame& a, const Frame& b) {
  double sum = 0;
  for (size_t i = 0; i < 3; i++) sum += (a.first[i] - b.first[i]) * (a.first[i] - b.first[i]);
  auto ra = KDL::Rotation::Quaternion(a.second[0], a.second[1], a.second[2], a.second[3]);
  auto rb = KDL::Rotation::Quaternion(b.second[0], b.second[1], b.second[2], b.second[3]);
  KDL::Vector v;
  auto delta = ra * rb.Inverse();
  auto angle = delta.GetRotAngle(v) / M_PI;
  sum += angle * angle;
  return sqrt(sum);
}

inline std::shared_ptr<spdlog::logger> setup_logger(std::vector<spdlog::sink_ptr> sinks,
                                                    const std::string& logger_name = "airbot") {
  auto logger = spdlog::get(logger_name);
  if (!logger) {
    if (sinks.size() > 0) {
      logger = std::make_shared<spdlog::logger>(logger_name, std::begin(sinks), std::end(sinks));
      spdlog::register_logger(logger);
    } else {
      logger = spdlog::stdout_color_mt(logger_name);
    }
  }
  return logger;
}

#endif
