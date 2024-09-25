#include "mpc_planner_modules/decomp_constraints.h"

#include <costmap_2d/costmap_2d_ros.h>

#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>

#include <ros_tools/profiling.h>
#include <ros_tools/visuals.h>
#include <ros_tools/spline.h>

#include <algorithm>

namespace MPCPlanner
{
  DecompConstraints::DecompConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "decomp_constraints")
  {
    LOG_INITIALIZE("Decomp Constraints");
    _decomp_util = std::make_unique<EllipsoidDecomp2D>();

    // Only look around for obstacles using a box with sides of width 2*range
    double range = CONFIG["decomp"]["range"].as<double>();
    _decomp_util->set_local_bbox(Vec2f(range, range));

    _occ_pos.reserve(1000); // Reserve some space for the occupied positions

    _n_discs = CONFIG["n_discs"].as<int>(); // Is overwritten to 1 for topology constraints

    _max_constraints = CONFIG["decomp"]["max_constraints"].as<int>();
    _a1.resize(_n_discs);
    _a2.resize(_n_discs);
    _b.resize(_n_discs);
    for (int d = 0; d < _n_discs; d++)
    {
      _a1[d].resize(CONFIG["N"].as<int>());
      _a2[d].resize(CONFIG["N"].as<int>());
      _b[d].resize(CONFIG["N"].as<int>());
      for (int k = 0; k < CONFIG["N"].as<int>(); k++)
      {
        _a1[d][k] = Eigen::ArrayXd(_max_constraints);
        _a2[d][k] = Eigen::ArrayXd(_max_constraints);
        _b[d][k] = Eigen::ArrayXd(_max_constraints);
      }
    }

    LOG_INITIALIZED();
  }

  void DecompConstraints::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;
    (void)module_data;

    PROFILE_SCOPE("DecompConstraints::Update");
    LOG_MARK("DecompConstraints::update");

    _dummy_b = state.get("x") + 100.;

    getOccupiedGridCells(data); // Retrieve occupied points from the costmap

    _decomp_util->set_obs(_occ_pos); // Set them

    // getPath(path);

    vec_Vec2f path;
    double s = state.get("spline");
    for (int k = 0; k < _solver->N; k++)
    {
      // Local path //
      // path.emplace_back(_solver->getEgoPrediction(k, "x"), _solver->getEgoPrediction(k, "y")); // k = 0 is initial state

      // Global (reference) path //
      auto path_pos = module_data.path->getPoint(s);
      path.emplace_back(path_pos(0), path_pos(1));

      double v = _solver->getEgoPrediction(k, "v"); // Use the predicted velocity

      s += v * _solver->dt;
    }
    _decomp_util->dilate(path, 0, false);

    _decomp_util->set_constraints(_constraints, 0.); // Map is already inflated
    _polyhedrons = _decomp_util->get_polyhedrons();

    int max_decomp_constraints = 0;

    for (int k = 0; k < _solver->N - 1; k++)
    {
      const auto &constraints = _constraints[k];
      max_decomp_constraints = std::max(max_decomp_constraints, (int)constraints.A_.rows());

      int i = 0;
      for (; i < std::min((int)constraints.A_.rows(), _max_constraints); i++)
      {
        if (constraints.A_.row(i).norm() < 1e-3 || constraints.A_(i, 0) != constraints.A_(i, 0)) // If zero or nan
        {
          break;
        }

        _a1[0][k + 1](i) = constraints.A_.row(i)[0];
        _a2[0][k + 1](i) = constraints.A_.row(i)[1];
        _b[0][k + 1](i) = constraints.b_(i);
      }

      for (; i < _max_constraints; i++)
      {
        _a1[0][k + 1](i) = _dummy_a1;
        _a2[0][k + 1](i) = _dummy_a2;
        _b[0][k + 1](i) = _dummy_b;
      }
    }

    if (max_decomp_constraints > _max_constraints)
      LOG_WARN("Maximum number of decomp util constraints exceeds specification: " << max_decomp_constraints << " > " << _max_constraints);

    LOG_MARK("DecompConstraints::update done");
  }

  bool DecompConstraints::getOccupiedGridCells(const RealTimeData &data)
  {
    PROFILE_FUNCTION();
    LOG_MARK("GetOccupiedGridCells");

    const auto &costmap = *data.costmap;

    // Store all occupied cells in the grid map
    _occ_pos.clear();
    double x, y;
    for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++)
    {
      for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++)
      {
        if (costmap.getCost(i, j) == costmap_2d::FREE_SPACE)
          continue;

        costmap.mapToWorld(i, j, x, y);
        // LOG_INFO("Obstacle at x = " << x << ", y = " << y);

        _occ_pos.emplace_back(x, y);
      }
    }
    // LOG_VALUE("Occupied cells", _occ_pos.size());

    return true;
  }

  void DecompConstraints::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {

    if (k == 0) // Dummies
    {
      for (int d = 0; d < _n_discs; d++)
      {
        setSolverParameterEgoDiscOffset(k, _solver->_params, data.robot_area[d].offset, d);

        int constraint_counter = 0;
        for (int i = 0; i < _max_constraints; i++)
        {
          setSolverParameterDecompA1(k, _solver->_params, _dummy_a1, constraint_counter); // These are filled from k = 1 - N
          setSolverParameterDecompA2(k, _solver->_params, _dummy_a2, constraint_counter);
          setSolverParameterDecompB(k, _solver->_params, _dummy_b, constraint_counter);
          constraint_counter++;
        }
      }

      return;
    }

    if (k == 1)
      LOG_MARK("DecompConstraints::setParameters");

    (void)module_data;
    int constraint_counter = 0; // Necessary for now to map the disc and obstacle index to a single index
    for (int d = 0; d < _n_discs; d++)
    {
      setSolverParameterEgoDiscOffset(k, _solver->_params, data.robot_area[d].offset, d);

      for (int i = 0; i < _max_constraints; i++)
      {
        setSolverParameterDecompA1(k, _solver->_params, _a1[d][k](i), constraint_counter); // These are filled from k = 1 - N
        setSolverParameterDecompA2(k, _solver->_params, _a2[d][k](i), constraint_counter);
        setSolverParameterDecompB(k, _solver->_params, _b[d][k](i), constraint_counter);
        constraint_counter++;
      }
    }
  }

  bool DecompConstraints::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    if (data.costmap == nullptr)
    {
      missing_data += "Costmap ";
      return false;
    }

    return true;
  }

  void DecompConstraints::projectToSafety(Eigen::Vector2d &pos)
  {
    // Too slow
    if (_occ_pos.empty()) // There is no anchor
      return;

    // Project to a collision free position if necessary, considering all the obstacles
    for (int iterate = 0; iterate < 1; iterate++) // At most 3 iterations
    {
      for (auto &obs : _occ_pos)
      {
        double radius = CONFIG["robot_radius"].as<double>() + 0.1;

        dr_projection_.douglasRachfordProjection(pos, obs, _occ_pos[0], radius, pos);
      }
    }
  }

  void DecompConstraints::visualize(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)data;
    (void)module_data;
    PROFILE_FUNCTION();

    bool visualize_points = false;

    auto &publisher = VISUALS.getPublisher("free_space");
    auto &polypoint = publisher.getNewPointMarker("CUBE");
    polypoint.setScale(0.1, 0.1, 0.1);
    polypoint.setColor(1, 0, 0, 1);

    auto &polyline = publisher.getNewLine();
    polyline.setScale(0.1, 0.1);
    for (int k = 0; k < _solver->N; k += CONFIG["visualization"]["draw_every"].as<int>())
    {
      const auto &poly = _polyhedrons[k];
      polyline.setColorInt(k, _solver->N);

      const auto vertices = cal_vertices(poly);
      if (vertices.size() < 2)
        continue;

      for (size_t i = 0; i < vertices.size(); i++)
      {
        if (visualize_points)
          polypoint.addPointMarker(Eigen::Vector3d(vertices[i](0), vertices[i](1), 0));

        if (i > 0)
        {
          polyline.addLine(
              Eigen::Vector3d(vertices[i - 1](0), vertices[i - 1](1), 0),
              Eigen::Vector3d(vertices[i](0), vertices[i](1), 0));
        }
      }
      polyline.addLine(Eigen::Vector3d(vertices.back()(0), vertices.back()(1), 0),
                       Eigen::Vector3d(vertices[0](0), vertices[0](1), 0));
    }

    publisher.publish();

    if (!CONFIG["debug_visuals"].as<bool>())
      return;

    LOG_MARK("DecompConstraints::Visualize");

    auto &map_publisher = VISUALS.getPublisher("map");
    auto &point = map_publisher.getNewPointMarker("CUBE");
    point.setScale(0.1, 0.1, 0.1);
    point.setColor(0, 0, 0, 1);

    for (auto &vec : _occ_pos)
    {
      point.addPointMarker(Eigen::Vector3d(vec.x(), vec.y(), 0));
    }
    map_publisher.publish();
  }

} // namespace MPCPlanner
