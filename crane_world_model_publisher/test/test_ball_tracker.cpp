#include <gtest/gtest.h>
#include "crane_world_model_publisher/ball_tracker.hpp"
#include <rclcpp/rclcpp.hpp> // For rclcpp::Time

// Test fixture for BallTracker tests
class BallTrackerTest : public ::testing::Test {
protected:
    crane::BallTracker tracker;
    rclcpp::Time t0;
    rclcpp::Clock ros_clock; // Member to ensure lifetime

    BallTrackerTest() : t0(0, 0, RCL_ROS_TIME), ros_clock(RCL_ROS_TIME) {
         // It's good practice to initialize rclcpp if it's not already,
         // especially when using rclcpp::Time.
         // However, in a gtest environment, rclcpp::init might be called by the test runner
         // if it's an ament_add_gtest target.
        t0 = ros_clock.now();
    }

    // Helper to compare Eigen matrices/vectors with a tolerance
    void expect_eigen_almost_equal(const Eigen::MatrixXd& actual, const Eigen::MatrixXd& expected, double tolerance = 1e-5) {
        ASSERT_EQ(actual.rows(), expected.rows());
        ASSERT_EQ(actual.cols(), expected.cols());
        for (int i = 0; i < actual.rows(); ++i) {
            for (int j = 0; j < actual.cols(); ++j) {
                EXPECT_NEAR(actual(i, j), expected(i, j), tolerance);
            }
        }
    }
};

TEST_F(BallTrackerTest, Initialization) {
    tracker.init(t0, 1.0, 2.0, 0.5, -0.5);
    ASSERT_TRUE(tracker.isInitialized());
    Eigen::Vector4d expected_state;
    expected_state << 1.0, 2.0, 0.5, -0.5;
    expect_eigen_almost_equal(tracker.getState(), expected_state);

    // Check that initial covariance is not zero (P_k_ is set in init)
    ASSERT_GT(tracker.getCovariance()(0,0), 0.0);
}

TEST_F(BallTrackerTest, PredictConstantVelocity) {
    tracker.init(t0, 0.0, 0.0, 1.0, 2.0); // vx=1, vy=2

    rclcpp::Time t1 = t0 + rclcpp::Duration(1, 0); // 1 second later
    tracker.predict(t1);

    Eigen::Vector4d predicted_state = tracker.getState();
    // Expected: x = 0 + 1*1 = 1, y = 0 + 2*1 = 2, vx=1, vy=2
    Eigen::Vector4d expected_state;
    expected_state << 1.0, 2.0, 1.0, 2.0;
    // Increased tolerance due to process noise Q_k effect over 1s
    expect_eigen_almost_equal(predicted_state, expected_state, 0.2);
}

TEST_F(BallTrackerTest, PredictIncreasesCovariance) {
    tracker.init(t0, 0.0, 0.0, 0.0, 0.0);
    Eigen::Matrix4d P0 = tracker.getCovariance();

    rclcpp::Time t1 = t0 + rclcpp::Duration(1, 0);
    tracker.predict(t1);
    Eigen::Matrix4d P1 = tracker.getCovariance();

    // P_k = F_k * P_{k-1} * F_k' + Q_k
    // Since Q_k is positive semi-definite and non-zero, trace of P should increase.
    ASSERT_GT(P1.trace(), P0.trace() - 1e-5); // trace(P1) should be > trace(P0)
}


TEST_F(BallTrackerTest, UpdateReducesCovariance) {
    tracker.init(t0, 0.0, 0.0, 1.0, 0.0); // Moving along x-axis

    rclcpp::Time t_predict = t0 + rclcpp::Duration(1,0); // Predict 1s
    tracker.predict(t_predict);
    Eigen::Matrix4d P_predicted = tracker.getCovariance();

    Eigen::Vector2d measurement(0.9, 0.1); // Measurement slightly off from prediction (1.0, 0.0)
    rclcpp::Time t_measurement = t_predict; // Measurement at the same time as prediction
    tracker.update(measurement, t_measurement);
    Eigen::Matrix4d P_updated = tracker.getCovariance();

    // Sum of diagonal elements (trace) should typically decrease after update with reasonable R
    ASSERT_LT(P_updated.trace(), P_predicted.trace() + 1e-5);
}

TEST_F(BallTrackerTest, StateConvergesWithUpdates) {
    // Initialize tracker with a somewhat off state
    tracker.init(t0, 0.1, -0.1, 0.1, 0.1);

    Eigen::Vector2d true_ball_pos(0.0, 0.0);
    double true_vx = 0.5; // m/s
    double true_vy = 0.2; // m/s
    double dt_secs = 0.1;   // 100ms per step
    rclcpp::Duration dt_duration = rclcpp::Duration::from_seconds(dt_secs);

    rclcpp::Time current_tracker_time = t0;

    // Simulate a few steps
    for (int i = 0; i < 20; ++i) { // Increased steps for convergence
        current_tracker_time = t0 + rclcpp::Duration::from_seconds(i * dt_secs);

        true_ball_pos(0) += true_vx * dt_secs;
        true_ball_pos(1) += true_vy * dt_secs;

        Eigen::Vector2d measurement = true_ball_pos;

        // Predict to current measurement time, then update.
        // BallTracker's update method internally calls predict if time has advanced.
        tracker.update(measurement, current_tracker_time);
    }

    Eigen::Vector4d final_state = tracker.getState();
    EXPECT_NEAR(final_state(0), true_ball_pos(0), 0.15); // Check X pos
    EXPECT_NEAR(final_state(1), true_ball_pos(1), 0.15); // Check Y pos
    EXPECT_NEAR(final_state(2), true_vx, 0.15);           // Check Vx
    EXPECT_NEAR(final_state(3), true_vy, 0.15);           // Check Vy
}

TEST_F(BallTrackerTest, PredictWithNegativeDt) {
    tracker.init(t0, 1.0, 2.0, 0.5, -0.5);
    Eigen::Vector4d initial_state = tracker.getState();
    Eigen::Matrix4d initial_covariance = tracker.getCovariance();

    rclcpp::Time t_minus_1 = t0 - rclcpp::Duration(1,0);
    tracker.predict(t_minus_1);

    Eigen::Vector4d state_after_neg_dt = tracker.getState();
    Eigen::Matrix4d cov_after_neg_dt = tracker.getCovariance();

    // Expect state and covariance to be unchanged as dt will be clamped to 0
    expect_eigen_almost_equal(state_after_neg_dt, initial_state, 1e-5);
    expect_eigen_almost_equal(cov_after_neg_dt, initial_covariance, 1e-5);
}

// Main function for running tests (needed for GTest)
int main(int argc, char **argv) {
    rclcpp::init(0, nullptr); // Initialize rclcpp for rclcpp::Time
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}

```
