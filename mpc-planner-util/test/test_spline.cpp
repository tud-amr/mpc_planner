#include <gtest/gtest.h>

#include <mpc-planner-util/spline.h>

using namespace MPCPlanner;

// Define a test fixture
class SplineTest : public ::testing::Test
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
TEST_F(SplineTest, TestName)
{
    auto x = {0.0, 1.0, 2.0, 3.0, 4.0};
    auto y = {1.0, 1.0, 1.0, 1.0, 1.0};

    Spline2D spline(x, y);
    ASSERT_TRUE(spline.numSegments() == 4);
    ASSERT_TRUE(spline.length() == 4.);

    ASSERT_TRUE(spline.getPoint(0)(0) == 0.);
    ASSERT_TRUE(spline.getPoint(0)(1) == 1.);
    ASSERT_TRUE(spline.getPoint(4)(0) == 4.);
    ASSERT_TRUE(spline.getPoint(4)(1) == 1.);

    ASSERT_TRUE(spline.getStartOfSegment(0) == 0.);
    ASSERT_TRUE(spline.getStartOfSegment(1) == 1.);
    ASSERT_TRUE(spline.getStartOfSegment(3) == 3.);

    int segment_out;
    double s_out;
    spline.findClosestPoint(Eigen::Vector2d(2.5, 3.0), segment_out, s_out);
    ASSERT_TRUE(segment_out == 2);
    ASSERT_TRUE(std::abs(s_out - 2.5) < 1e-5);
}

// Run all the tests
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
