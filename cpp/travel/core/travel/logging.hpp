#ifndef TRAVEL_LOGGING_HPP
#define TRAVEL_LOGGING_HPP

// Logging facade for the TRAVEL core library.
//
// By default the macros expand to printf-based stderr/stdout output so the
// core stays free of ROS dependencies and can be consumed from a pure C++
// example, a Python pybind11 module, or any non-ROS build.
//
// The ROS wrapper (`ros/`) defines TRAVEL_USE_ROS_LOGGING via
// target_compile_definitions so the same printf-style call sites in the
// core dispatch to ROS_INFO/ROS_WARN/ROS_ERROR transparently.

#if defined(TRAVEL_USE_ROS_LOGGING)
  #include <ros/ros.h>
  #define TRAVEL_LOG_INFO(...)  ROS_INFO(__VA_ARGS__)
  #define TRAVEL_LOG_WARN(...)  ROS_WARN(__VA_ARGS__)
  #define TRAVEL_LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
#else
  #include <cstdio>
  #define TRAVEL_LOG_INFO(...)  do { std::fprintf(stdout, "[TRAVEL][INFO] ");  std::fprintf(stdout, __VA_ARGS__); std::fprintf(stdout, "\n"); } while (0)
  #define TRAVEL_LOG_WARN(...)  do { std::fprintf(stderr, "[TRAVEL][WARN] ");  std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
  #define TRAVEL_LOG_ERROR(...) do { std::fprintf(stderr, "[TRAVEL][ERROR] "); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#endif

#endif  // TRAVEL_LOGGING_HPP
