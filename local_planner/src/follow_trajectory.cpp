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
#include <rrt_planner/rrt_planner.h>
#include <rrt_planner/GetPlan.h>

class TrajectoryGenerator
{
public:
    ros::NodeHandle nh_;

    ros::Publisher arm_pub_;

    ros::Subscriber autopilot_feedback_sub_;

    ros::ServiceClient planner_client;

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

        arm_pub_ = nh_.advertise<std_msgs::Bool>("hummingbird/bridge/arm", 1);
        planner_client = nh_.serviceClient<rrt_planner::GetPlan>("rrt_planner_server");
    }

    ~TrajectoryGenerator() {}

    void run()
    {
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

        // Call RRT planner service
        rrt_planner::GetPlan planner_call = rrt_planner::GetPlan();
        planner_call.request.start.pose.position.x = start_state.position(0);
        planner_call.request.start.pose.position.y = start_state.position(1);

        // TODO: get goal from somewhere
        planner_call.request.goal.pose.position.x = 7.0;
        planner_call.request.goal.pose.position.y = 5.0;

        // Send Obstacle IDs to avoid. Expects a int16[] array
        std::vector<int16_t> obstacles = {0, 1, 2, 3};
        planner_call.request.obstacle_ids.data = obstacles;

        ros::service::waitForService("rrt_planner_server");
        if (!planner_client.call(planner_call))
        {
            ROS_ERROR("Failed to call planner service");
            return;
        }

        // Convert nav path to waypoints
        int planlength = planner_call.response.plan.poses.size();
        std::vector<Eigen::Vector3d> waypoints;
        for (int i = 0; i < planlength; i++)
        {
            Eigen::Vector3d waypoint;
            waypoint << planner_call.response.plan.poses[i].pose.position.x, planner_call.response.plan.poses[i].pose.position.y, start_state.position(2);
            waypoints.push_back(waypoint);
        }

        // End state
        quadrotor_common::TrajectoryPoint end_state;
        end_state.position = waypoints.back();
        end_state.heading = 0.0;

        Eigen::VectorXd initial_segment_times =
            Eigen::VectorXd::Ones(int(waypoints.size())+1);
        polynomial_trajectories::PolynomialTrajectorySettings
            trajectory_settings;
        trajectory_settings.continuity_order = 4;
        Eigen::VectorXd minimization_weights(5);
        minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
        trajectory_settings.minimization_weights = minimization_weights;
        trajectory_settings.polynomial_order = 11;
        trajectory_settings.way_points = waypoints;

        quadrotor_common::Trajectory traj = trajectory_generation_helper::
            polynomials::generateMinimumSnapTrajectory(
                initial_segment_times, start_state, end_state, trajectory_settings, max_vel,
                max_thrust, max_roll_pitch_rate, kExecLoopRate_);

        // print trajectory before sending
        // traj.points is a list
        for(auto point : traj.points){
            ROS_INFO("Trajectory point: %f, %f, %f", point.position(0), point.position(1), point.position(2));
        }

        autopilot_helper_.sendTrajectory(traj);

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