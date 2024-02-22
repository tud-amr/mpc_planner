#include <gtest/gtest.h>

// Include the header file for the class you want to test
#include "mpc-planner-solver/state.h"
#include "mpc-planner-solver/solver_interface.h"

#include <mpc-planner-util/parameters.h>

#include <filesystem>

using namespace MPCPlanner;

// Define a test fixture
class StateTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Set up any necessary resources before each test
    }

    void TearDown() override
    {
        // Clean up any resources after each test
    }

    // Declare any member variables or helper functions that you need
};

// Define a test fixture
class SolverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        std::string path = std::filesystem::path(__FILE__).parent_path().string() + "/../../mpc-planner-jackal/src/src";
        path = SYSTEM_CONFIG_PATH(path, "settings");
        Configuration::getInstance().initialize(path);

        // Set up any necessary resources before each test
    }

    void TearDown() override
    {
        // Clean up any resources after each test
    }

    // Declare any member variables or helper functions that you need
};

// Define a test case
TEST_F(StateTest, TestName)
{
    // Create an instance of your class
    State state;

    state.set("x", 1.5);
    state.set("y", 3.5);

    // Perform any necessary assertions
    ASSERT_TRUE(state.get("y") == 3.5);
    ASSERT_TRUE(state.get("x") == 1.5);
    ASSERT_TRUE(state.getPos()(0) == 1.5);
    ASSERT_TRUE(state.getPos()(1) == 3.5);
}

TEST_F(SolverTest, TestName)
{
    Solver solver;

    ASSERT_TRUE(solver.nu + solver.nx == solver.nvar);
    ASSERT_TRUE(solver.npar > 0);
    ASSERT_TRUE(solver.dt > 0.);

    // Test parameters
    for (int k = 0; k < solver.N; k++)
        solver.setParameter(k, "reference_velocity", 1.);
    for (int k = 0; k < solver.N; k++)
        ASSERT_TRUE(solver.getParameter(k, "reference_velocity") == 1.);

    // Test Xinit
    solver.setXinit("x", 5.4);
    solver.setXinit("y", 1.4);
    double sum_init = 0.;
    for (unsigned int i = 0; i < solver.nx; i++)
        sum_init += solver._params.xinit[i];
    ASSERT_TRUE(std::abs(sum_init - 6.8) < 1e-5);

    State state;
    state.set("x", 4.4);
    state.set("y", 1.2);
    solver.setXinit(state);
    sum_init = 0.;
    for (unsigned int i = 0; i < solver.nx; i++)
        sum_init += solver._params.xinit[i];
    ASSERT_TRUE(std::abs(sum_init - 5.6) < 1e-5);

    // Set to zero from here on out
    state = State();
    solver.setXinit(state);

    // Test the indices of the prediction
    // Step 1: put a fake plan into x0
    for (int k = 0; k < solver.N; k++)
    {
        solver.setEgoPrediction(k, "x", k * 1.);
        solver.setEgoPrediction(k, "y", 0.);
    }
    // Verify that set / get works
    for (int k = 0; k < solver.N; k++)
    {
        ASSERT_TRUE(solver.getEgoPrediction(k, "x") == k * 1);
        ASSERT_TRUE(solver.getEgoPrediction(k, "y") == 0);
    }

    // Put things in the output as FORCES would (first is the initial state)
    /** @note Assumes N = 20*/
    int x_index = solver._model_map["x"][1].as<int>();
    solver._output.x01[x_index] = 0.0; // Initial state
    solver._output.x02[x_index] = 1.0; // Predicted initial state at the next control instance
    solver._output.x03[x_index] = 2.0; // First predicted state of the next control instance

    // In the next control iteration, the initial state turns out to be 0.8
    state.set("x", 0.8);
    solver.setWarmstart(state);                          // Set the warmstart based on the output
    ASSERT_TRUE(solver.getEgoPrediction(0, "x") == 0.8); // Check if the initial state was added as initial prediction
    ASSERT_TRUE(solver.getEgoPrediction(1, "x") == 2.0); // Check if the first prediction was correctly copied from the output

    Solver solver2(1);
    solver2.setParameter(0, "reference_velocity", 3.);
    ASSERT_TRUE(solver._solver_id != solver2._solver_id);
    solver2 = solver;
    ASSERT_TRUE(solver2.getParameter(0, "reference_velocity") == 1.);
}

// Run all the tests
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
