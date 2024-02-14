#include <gtest/gtest.h>

// Include the header file for the class you want to test
#include "mpc-planner-solver/state.h"

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
}

// Run all the tests
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
