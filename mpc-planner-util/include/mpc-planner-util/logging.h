
#ifndef __MPC_PLANNER_LOGGING_H_
#define __MPC_PLANNER_LOGGING_H_

// #ifndef MPC_PLANNER_ROS
#define MPC_PLANNER_ROS 1
// #endif

#if MPC_PLANNER_ROS == 1
#include <ros/ros.h>
#define LOG_INFO(...) ROS_INFO_STREAM(__VA_ARGS__)
#define LOG_WARN(...) ROS_WARN_STREAM("\033[33m" << __VA_ARGS__ << "\033[0m")
#define LOG_ERROR(...) ROS_ERROR_STREAM(__VA_ARGS__)
#define LOG_DEBUG(...) ROS_DEBUG_STREAM(__VA_ARGS__)
#define LOG_INFO_THROTTLE(rate, ...) ROS_INFO_STREAM_THROTTLE(rate, __VA_ARGS__)
#define LOG_WARN_THROTTLE(rate, ...) ROS_WARN_STREAM_THROTTLE(rate, "\033[33m" << __VA_ARGS__ << "\033[0m")
#define LOG_ERROR_THROTTLE(rate, ...) ROS_ERROR_STREAM_THROTTLE(rate, __VA_ARGS__)
#define LOG_DEBUG_THROTTLE(rate, ...) ROS_DEBUG_STREAM_THROTTLE(rate, __VA_ARGS__)
#elif MPC_PLANNER_ROS == 2
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#define LOGGING_NAME std::filesystem::path(__FILE__).filename().replace_extension("").string()
#define LOG_INFO(...) RCLCPP_INFO_STREAM(rclcpp::get_logger(LOGGING_NAME), __VA_ARGS__)
#define LOG_WARN(...) RCLCPP_WARN_STREAM(rclcpp::get_logger(LOGGING_NAME), "\033[33m" << __VA_ARGS__ << "\033[0m")
#define LOG_ERROR(...) RCLCPP_ERROR_STREAM(rclcpp::get_logger(LOGGING_NAME), __VA_ARGS__)
#define LOG_DEBUG(...) RCLCPP_DEBUG_STREAM(rclcpp::get_logger(LOGGING_NAME), __VA_ARGS__)
#define LOG_INFO_THROTTLE(rate, ...) RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger(LOGGING_NAME), rate, __VA_ARGS__)
#define LOG_WARN_THROTTLE(rate, ...) RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger(LOGGING_NAME), rate, "\033[33m" << __VA_ARGS__ << "\033[0m")
#define LOG_ERROR_THROTTLE(rate, ...) RCLCPP_ERROR_STREAM_THROTTLE(rclcpp::get_logger(LOGGING_NAME), rate, __VA_ARGS__)
#define LOG_DEBUG_THROTTLE(rate, ...) RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger(LOGGING_NAME), rate, __VA_ARGS__)
#else
#include <iostream>
#define LOG_INFO(...) std::cout << __VA_ARGS__ << std::endl
#define LOG_WARN(...) std::cerr << "\033[33mWarning: " << __VA_ARGS__ << "\033[0m" << std::endl
#define LOG_ERROR(...) std::cerr << "Error: " << __VA_ARGS__ << std::endl
#define LOG_DEBUG(...) std::cout << "Debug: " << __VA_ARGS__ << std::endl
#define LOG_INFO_THROTTLE(rate, ...) std::cout << __VA_ARGS__ << std::endl
#define LOG_WARN_THROTTLE(rate, ...) std::cerr << "\033[33mWarning: " << __VA_ARGS__ << "\033[0m" << std::endl
#define LOG_ERROR_THROTTLE(rate, ...) std::cerr << "Error: " << __VA_ARGS__ << std::endl
#define LOG_DEBUG_THROTTLE(rate, ...) std::cout << "Debug: " << __VA_ARGS__ << std::endl
#endif

#define LOG_VALUE(name, value) LOG_INFO("\033[1m" << name << ":\033[0m " << value)
#define LOG_VALUE_DEBUG(name, value) LOG_DEBUG("\033[1m" << name << ":\033[0m " << value)
#define LOG_DIVIDER() LOG_INFO("========================================")

#endif // LOGGING_H
