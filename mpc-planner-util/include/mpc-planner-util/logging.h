
#ifndef LOGGING_H
#define LOGGING_H

// #ifndef MPC_PLANNER_ROS
#define MPC_PLANNER_ROS 2
// #endif

#if MPC_PLANNER_ROS == 1
#include <ros/console.h>
#define LOG_INFO(...) ROS_INFO_STREAM(__VA_ARGS__)
#define LOG_WARN(...) ROS_WARN_STREAM(__VA_ARGS__)
#define LOG_ERROR(...) ROS_ERROR_STREAM(__VA_ARGS__)
#define LOG_INFO_THROTTLE(rate, ...) ROS_INFO_STREAM_THROTTLE(rate, __VA_ARGS__)
#define LOG_WARN_THROTTLE(rate, ...) ROS_WARN_STREAM_THROTTLE(rate, __VA_ARGS__)
#define LOG_ERROR_THROTTLE(rate, ...) ROS_ERROR_STREAM_THROTTLE(rate, __VA_ARGS__)
#elif MPC_PLANNER_ROS == 2
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#define LOGGING_NAME std::filesystem::path(__FILE__).filename().replace_extension("").string()
#define LOG_INFO(...) RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGING_NAME), __VA_ARGS__)
#define LOG_WARN(...) RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGING_NAME), __VA_ARGS__)
#define LOG_ERROR(...) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGING_NAME), __VA_ARGS__)
#define LOG_INFO_THROTTLE(rate, ...) RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger(LOGGING_NAME), rate, __VA_ARGS__)
#define LOG_WARN_THROTTLE(rate, ...) RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger(LOGGING_NAME), rate, __VA_ARGS__)
#define LOG_ERROR_THROTTLE(rate, ...) RCLCPP_ERROR_STREAM_THROTTLE(rclcpp::get_logger(LOGGING_NAME), rate, __VA_ARGS__)
#else
#include <iostream>
#define LOG_INFO(...) std::cout << __VA_ARGS__ << std::endl
#define LOG_WARN(...) std::cerr << "Warning: " << __VA_ARGS__ << std::endl
#define LOG_ERROR(...) std::cerr << "Error: " << __VA_ARGS__ << std::endl
#define LOG_INFO_THROTTLE(rate, ...) std::cout << __VA_ARGS__ << std::endl
#define LOG_WARN_THROTTLE(rate, ...) std::cerr << "Warning: " << __VA_ARGS__ << std::endl
#define LOG_ERROR_THROTTLE(rate, ...) std::cerr << "Error: " << __VA_ARGS__ << std::endl
#endif

#define LOG_VALUE(name, value) LOG_INFO("\033[1m" << name << ":\033[0m " << value)

#endif // LOGGING_H
