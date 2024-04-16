#include <mpc_planner/experiment_util.h>

#include <mpc_planner_solver/solver_interface.h>
#include <mpc_planner_solver/state.h>
#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_util/parameters.h>

#include <ros_tools/logging.h>
#include <ros_tools/profiling.h>

#include <filesystem>

namespace MPCPlanner
{
    ExperimentUtil::ExperimentUtil()
    {
        _save_folder = CONFIG["recording"]["folder"].as<std::string>();
        _save_file = CONFIG["recording"]["file"].as<std::string>();
        LOG_VALUE("Planner Save File", _save_folder + "/" + _save_file + ".txt");

        _data_saver = std::make_unique<RosTools::DataSaver>();
    }

    void ExperimentUtil::update(const State &state, std::shared_ptr<Solver> solver, const RealTimeData &data)
    {
        // Save data of this control iteration
        LOG_MARK("ExperimentUtil::SaveData()");

        // Don't export if the obstacles aren't ready
        if (data.dynamic_obstacles.size() == 0)
        {
            LOG_INFO_THROTTLE(5000., "Not exporting data: Obstacles not yet received.");
            return;
        }

        // SAVE VEHICLE DATA
        _data_saver->AddData("vehicle_pose", state.getPos());
        _data_saver->AddData("vehicle_orientation", state.get("psi"));

        // Save the planned trajectory
        for (int k = 0; k < CONFIG["N"].as<int>(); k++)
            _data_saver->AddData("vehicle_plan_" + std::to_string(k), solver->getEgoPredictionPosition(k));

        // SAVE OBSTACLE DATA
        for (size_t v = 0; v < data.dynamic_obstacles.size(); v++)
        {
            auto &obstacle = data.dynamic_obstacles[v];

            // CARLA / Real Jackal
            if (obstacle.index != -1)
            {
                _data_saver->AddData("obstacle_map_" + std::to_string(v), obstacle.index);
                _data_saver->AddData("obstacle_" + std::to_string(v) + "_pose", obstacle.position);
                _data_saver->AddData("obstacle_" + std::to_string(v) + "_orientation", obstacle.angle);
            }

            // DISCS (assume only one disc)
            _data_saver->AddData("disc_" + std::to_string(0) + "_pose", obstacle.position);
            _data_saver->AddData("disc_" + std::to_string(0) + "_radius", obstacle.radius);
            _data_saver->AddData("disc_" + std::to_string(0) + "_obstacle", v);
        }
        _data_saver->AddData("max_intrusion", data.intrusion);
        _data_saver->AddData("metric_collisions", int(data.intrusion > 0.));

        // TIME KEEPING
        _data_saver->AddData("iteration", _control_iteration);
        _control_iteration++;
    }

    void ExperimentUtil::exportData()
    {
        _data_saver->SaveData(_save_folder, _save_file);
    }

    void ExperimentUtil::onTaskComplete(bool objective_reached)
    {
        // Add the control iteration where the reset was triggered - This divides the saved data!
        _data_saver->AddData("reset", _control_iteration);

        // Add the duration (assume control frequency is constant)
        _data_saver->AddData(
            "metric_duration",
            (_control_iteration - _iteration_at_last_reset) * (1.0 / CONFIG["control_frequency"].as<double>()));

        _data_saver->AddData("metric_completed", (int)(objective_reached));
        _iteration_at_last_reset = _control_iteration;

        _experiment_counter++;

        // Save data to FILE when a number of experiments have been completed
        int num_experiments = CONFIG["recording"]["num_experiments"].as<int>();
        if (_experiment_counter % num_experiments == 0 && _experiment_counter > 0)
            exportData();

        // Save profiling data before we crash the controller
        if (_experiment_counter >= num_experiments)
        {
            RosTools::Instrumentor::Get().EndSession();
            LOG_SUCCESS("Completed " << num_experiments << " experiments.");
        }
        else
        {
            LOG_DIVIDER();
            LOG_INFO("Starting experiment " << _experiment_counter + 1 << " / " << num_experiments);
        }
        ROSTOOLS_ASSERT(_experiment_counter < num_experiments, "Stopping the planner.");
    }
}