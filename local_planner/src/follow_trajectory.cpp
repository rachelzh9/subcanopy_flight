#include <vector>

#include <autopilot/autopilot_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/ros.h>
#include <autopilot/autopilot_states.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>

class TrajectoryGenerator
{
public:
    ros::NodeHandle nh_;

    ros::Publisher arm_pub_;

    ros::Subscriber autopilot_feedback_sub_;

    autopilot_helper::AutoPilotHelper autopilot_helper_;
    bool executing_trajectory_;

    // Performance metrics variables
    double sum_position_error_squared_;
    double max_position_error_;
    double sum_thrust_direction_error_squared_;
    double max_thrust_direction_error_;

    // Constants
    static constexpr double kExecLoopRate_ = 50.0;

    TrajectoryGenerator()
        : executing_trajectory_(false),
          sum_position_error_squared_(0.0),
          max_position_error_(0.0),
          sum_thrust_direction_error_squared_(0.0),
          max_thrust_direction_error_(0.0)
    {

        arm_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1);
    }

    ~TrajectoryGenerator() {}

    void run()
    {
        ros::Duration(5.0).sleep();
        ROS_INFO("Running trajectory node");

        ros::Rate command_rate(kExecLoopRate_);

        // Make sure everything is up and running
        // Wait for Autopilot feedback with assert
        autopilot_helper_.waitForAutopilotFeedback(10.0, kExecLoopRate_);

        ROS_INFO("Arming bridge");

        // Arm bridge
        std_msgs::Bool arm_msg;
        arm_msg.data = true;
        arm_pub_.publish(arm_msg);

        ///////////////
        // Check off command
        ///////////////

        // Takeoff for real
        autopilot_helper_.sendStart();

        // Wait for autopilot to go to hover
        autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0, kExecLoopRate_);

        // Generate trajectory, sample it and send it as reference states
        const double max_vel = 2.0;
        const double max_thrust = 15.0;
        const double max_roll_pitch_rate = 0.5;

        quadrotor_common::TrajectoryPoint start_state;
        start_state.position = autopilot_helper_.getCurrentReferenceState().position;
        start_state.heading = 0.0;

        ROS_INFO("Starting trajectory control");
        ros::Duration(3.0).sleep();

        ///////////////
        // Check trajectory control
        ///////////////

        // Generate trajectories and send them as complete trajectories
        // One polynomial to enter a ring and a ring to check execution of
        // consecutive trajectories

        // Ring trajectory with enter segment
        std::vector<Eigen::Vector3d>
            way_points;
        way_points.push_back(Eigen::Vector3d(-0.5, 0.0, 1.5));
        way_points.push_back(Eigen::Vector3d(1.5, -1.5, 0.6));
        way_points.push_back(Eigen::Vector3d(3.5, 0.0, 2.0));
        way_points.push_back(Eigen::Vector3d(1.5, 2.0, 0.6));

        Eigen::VectorXd initial_ring_segment_times =
            Eigen::VectorXd::Ones(int(way_points.size()));
        polynomial_trajectories::PolynomialTrajectorySettings
            ring_trajectory_settings;
        ring_trajectory_settings.continuity_order = 4;
        Eigen::VectorXd minimization_weights(5);
        minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
        ring_trajectory_settings.minimization_weights = minimization_weights;
        ring_trajectory_settings.polynomial_order = 11;
        ring_trajectory_settings.way_points = way_points;

        quadrotor_common::Trajectory ring_traj = trajectory_generation_helper::
            polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
                initial_ring_segment_times, ring_trajectory_settings, max_vel,
                max_thrust, max_roll_pitch_rate, kExecLoopRate_);

        polynomial_trajectories::PolynomialTrajectorySettings
            enter_trajectory_settings = ring_trajectory_settings;
        enter_trajectory_settings.way_points.clear();

        quadrotor_common::Trajectory enter_traj =
            trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
                Eigen::VectorXd::Ones(1), start_state, ring_traj.points.front(),
                enter_trajectory_settings, 1.03 * max_vel, 1.03 * max_thrust,
                max_roll_pitch_rate, kExecLoopRate_);

        trajectory_generation_helper::heading::addConstantHeadingRate(
            start_state.heading, 0.0, &enter_traj);
        trajectory_generation_helper::heading::addConstantHeadingRate(0.0, M_PI,
                                                                      &ring_traj);

        autopilot_helper_.sendTrajectory(enter_traj);
        autopilot_helper_.sendTrajectory(ring_traj);

        // Check if autopilot goes to trajectory control state
        autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::TRAJECTORY_CONTROL, 5.0, kExecLoopRate_);

        // Wait for autopilot to go back to hover
        autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 30.0, kExecLoopRate_);
        ros::Duration(3.0).sleep();

        ///////////////
        // Check landing
        ///////////////

        // Land
        autopilot_helper_.sendLand();

        // Wait for autopilot to go to land
        autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::LAND, 10.0, kExecLoopRate_);

        // Wait for autopilot to go to off
        autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::OFF, 10.0, kExecLoopRate_);

        autopilot_helper_.sendOff();
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "follow_trajectory");
    TrajectoryGenerator tg;
    tg.run();
    ros::spin();
    return 0;
}