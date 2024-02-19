#ifndef __PLANNER_PROFILING_H__
#define __PLANNER_PROFILING_H__

#include <iostream>

#include <string>
#include <chrono>

namespace MPCPlanner
{
    // Use as static to print average run time
    class Benchmarker
    {
    public:
        Benchmarker(const std::string &name)
        {
            name_ = name;
            running_ = false;
        }

        // Print results on destruct
        ~Benchmarker()
        {
            double average_run_time = total_duration_ / ((double)total_runs_) * 1000.0;

            std::cout << "Timing Results for [" << name_ << "]" << std::endl;
            std::cout << "Average (ms): " << average_run_time << std::endl;
            std::cout << "Max (ms)" << max_duration_ * 1000.0 << std::endl;
        }

        void start()
        {
            running_ = true;
            start_time_ = std::chrono::system_clock::now();
        }

        double stop()
        {
            if (!running_)
                return 0.0;

            auto end_time = std::chrono::system_clock::now();
            std::chrono::duration<double> current_duration = end_time - start_time_;

            if (current_duration.count() < min_duration_)
                min_duration_ = current_duration.count();

            if (current_duration.count() > max_duration_)
                max_duration_ = current_duration.count();

            total_duration_ += current_duration.count();
            total_runs_++;
            running_ = false;

            last_ = current_duration.count();
            return last_;
        }

    private:
        std::chrono::system_clock::time_point start_time_;

        double total_duration_ = 0.0;
        double max_duration_ = -1.0;
        double min_duration_ = 99999.0;

        double last_ = -1.0;

        int total_runs_ = 0;

        std::string name_;
        bool running_ = false;
    };
}
#endif // PROFILING_H
