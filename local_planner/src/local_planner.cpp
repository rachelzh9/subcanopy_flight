#include <vector>
#include <autopilot/autopilot_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <autopilot/autopilot_states.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>
#include <rrt_planner/rrt_planner.h>
#include <rrt_planner/GetRRTPlan.h>

#define WAYPOINT_HORIZON 5
#define REACH_WAYPOINT_RADIUS 0.2

class LocalPlanner
{
public:
    ros::NodeHandle nh_;

    ros::Publisher arm_pub_;
    ros::Publisher snap_traj_pub_;
    ros::Publisher rrt_traj_pub_;

    ros::Subscriber goal_sub_;

    ros::ServiceClient planner_client;

    autopilot_helper::AutoPilotHelper autopilot_helper_;
    bool executing_trajectory_;

    geometry_msgs::PoseStamped goal_pose_;
    bool goal_received_;
    bool goal_reached_;

    polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings;

    // Performance metrics variables
    double sum_position_error_squared_;
    double max_position_error_;
    double sum_thrust_direction_error_squared_;
    double max_thrust_direction_error_;

    int waypoint_horizon_;
    double reach_waypoint_radius_;

    const double max_vel = 2.0;
    const double max_thrust = 15.0;
    const double max_roll_pitch_rate = 0.5;
    static constexpr double kExecLoopRate_ = 50.0;

    LocalPlanner()
        : executing_trajectory_(false),
          sum_position_error_squared_(0.0),
          max_position_error_(0.0),
          sum_thrust_direction_error_squared_(0.0),
          max_thrust_direction_error_(0.0),
          goal_received_(false),
          goal_reached_(false)
    {

        arm_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &LocalPlanner::goalCallback, this);  // goal is in global namespace
        snap_traj_pub_ = nh_.advertise<nav_msgs::Path>("snap_trajectory", 1);
        rrt_traj_pub_ = nh_.advertise<nav_msgs::Path>("rrt_trajectory", 1);
        planner_client = nh_.serviceClient<rrt_planner::GetRRTPlan>("/rrt_planner_server");  // planner is in global namespace

        waypoint_horizon_ = nh_.param<int>("waypoint_horizon", WAYPOINT_HORIZON);
        reach_waypoint_radius_ = nh_.param<double>("reach_waypoint_radius", REACH_WAYPOINT_RADIUS);

        trajectory_settings.continuity_order = 4;
        trajectory_settings.polynomial_order = 11;

        if (!autopilot_helper_.waitForAutopilotFeedback(10.0, kExecLoopRate_))
        {
            ROS_ERROR("Autopilot feedback not received. Is the autopilot running?");
        }

        ROS_INFO("Arming bridge");
        std_msgs::Bool arm_msg;
        arm_msg.data = true;
        arm_pub_.publish(arm_msg);
        ros::Duration(1.0).sleep();

        // Takeoff
        autopilot_helper_.sendStart();

        autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 10.0, kExecLoopRate_);
    }

    ~LocalPlanner() {}

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_INFO("Goal received");
        if (!goal_received_){
            goal_received_ = true;
        }
        else{
            ROS_INFO("Already have a goal. Will execute new goal when current goal is reached.");
        }
        goal_pose_ = *msg;
    }

    nav_msgs::Path getPlan(quadrotor_common::TrajectoryPoint start_state, geometry_msgs::PoseStamped goal_state)
    {
        rrt_planner::GetRRTPlan planner_call = rrt_planner::GetRRTPlan();
        planner_call.request.start.pose.position.x = start_state.position(0);
        planner_call.request.start.pose.position.y = start_state.position(1);

        planner_call.request.goal.pose.position.x = goal_state.pose.position.x;
        planner_call.request.goal.pose.position.y = goal_state.pose.position.y;

        // If empty, all obstacles are avoided
        std::vector<int16_t> obstacles = {};
        planner_call.request.obstacle_ids.data = obstacles;

        ros::service::waitForService("/rrt_planner_server");
        if (!planner_client.call(planner_call)){
            ROS_ERROR("Failed to call planner service");
            return nav_msgs::Path();
        }

        return planner_call.response.plan;
    }

    bool reachedWaypoint(Eigen::Vector3d waypoint)
    {
        Eigen::Vector3d current_position = autopilot_helper_.getCurrentReferenceState().position;
        double distance = (current_position - waypoint).norm();
        if (distance < reach_waypoint_radius_)
            return true;
        return false;
    }

    void executeFullTrajectory(quadrotor_common::TrajectoryPoint start_state, nav_msgs::Path plan)
    {
        std::vector<Eigen::Vector3d> waypoints = pathToWaypoints(start_state, plan);

        quadrotor_common::TrajectoryPoint end_state;
        end_state.position = waypoints.back();
        end_state.heading = 0.0;

        // remove start and end states from waypoints
        waypoints.erase(waypoints.begin());
        waypoints.pop_back();

        std::vector<Eigen::Vector3d> waypoints_sampled;
        for (int i=0; (int)i<waypoints.size(); i+=5) {
            waypoints_sampled.push_back(waypoints[i]);
        }

        Eigen::VectorXd initial_segment_times = Eigen::VectorXd::Ones(int(waypoints_sampled.size())+1);
        trajectory_settings.way_points = waypoints_sampled;

        Eigen::VectorXd minimization_weights(5);
        minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
        trajectory_settings.minimization_weights = minimization_weights;

        ROS_INFO("Executing trajectory with %d waypoints", int(waypoints_sampled.size()));
        
        quadrotor_common::Trajectory traj = trajectory_generation_helper::
            polynomials::generateMinimumSnapTrajectoryWithSegmentRefinement(
                initial_segment_times, start_state, end_state, trajectory_settings, max_vel,
                max_thrust, max_roll_pitch_rate, kExecLoopRate_);

        if(traj.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED){
                ROS_WARN("Failed to generate min snap trajectory with waypoints. Attempting to generate without waypoints.");
                trajectory_settings.way_points.clear();
                initial_segment_times = Eigen::VectorXd::Ones(1);
                traj = trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
                    initial_segment_times, start_state, end_state, trajectory_settings, max_vel,
                    max_thrust, max_roll_pitch_rate, kExecLoopRate_);
        }
        
        autopilot_helper_.sendTrajectory(traj);
        minSnapTrajPublisher(traj);

        goal_reached_ = true;

    }

    void executeTrajectoryWithWaypointHorizon(quadrotor_common::TrajectoryPoint start_state, nav_msgs::Path plan)
    {
        std::vector<Eigen::Vector3d> waypoints = pathToWaypoints(start_state, plan);

        Eigen::VectorXd minimization_weights(5);
        minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
        trajectory_settings.minimization_weights = minimization_weights;

        ROS_INFO("Executing trajectory with %d waypoints", (int)waypoints.size());

        // we want to loop through the waypoints and generate a trajectory for waypoint_horizon_ waypoints at a time
        for(int i = 0; i < waypoints.size()-1; i+=waypoint_horizon_){

            std::vector<Eigen::Vector3d> waypoints_subset;
            int j = i;
            for (j; j<i+waypoint_horizon_ && j<waypoints.size(); j++) waypoints_subset.push_back(waypoints[j]);
            // auto end_horizon = (waypoints.begin()+i+waypoint_horizon_-1 >= waypoints.end()) ? waypoints.end() : waypoints.begin()+i+waypoint_horizon_;
            ROS_INFO("start: %d, end: %d", i, j-1);
            ROS_INFO("x: %f, y: %f", waypoints_subset[0][0], waypoints_subset[0](1));
            ROS_INFO("x: %f, y: %f", waypoints_subset.back()[0], waypoints_subset.back()[1]);
            // waypoints_subset.assign(waypoints.begin() + i, end_horizon);

            start_state.position = autopilot_helper_.getCurrentReferenceState().position;
            start_state.heading = autopilot_helper_.getCurrentReferenceState().heading;

            quadrotor_common::TrajectoryPoint end_state;
            end_state.position = waypoints_subset.back();
            end_state.heading = 0.0;

            // remove start and end states from waypoints_subset
            waypoints_subset.erase(waypoints_subset.begin());
            waypoints_subset.pop_back();
            
            Eigen::VectorXd initial_segment_times = Eigen::VectorXd::Ones(int(waypoints_subset.size())+1);
            trajectory_settings.way_points.clear();
            if ((int)waypoints_subset.size() > 1) trajectory_settings.way_points = waypoints_subset;

            quadrotor_common::Trajectory traj;

            traj = trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
                    initial_segment_times, start_state, end_state, trajectory_settings, max_vel,
                    max_thrust, max_roll_pitch_rate, kExecLoopRate_);
        
            if(traj.trajectory_type == quadrotor_common::Trajectory::TrajectoryType::UNDEFINED){
                ROS_WARN("Failed to generate min snap trajectory with waypoints. Attempting to generate without waypoints.");
                trajectory_settings.way_points.clear();
                initial_segment_times = Eigen::VectorXd::Ones(1);
                traj = trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
                    initial_segment_times, start_state, end_state, trajectory_settings, max_vel,
                    max_thrust, max_roll_pitch_rate, kExecLoopRate_);
            }
            
            autopilot_helper_.sendTrajectory(traj);
            minSnapTrajPublisher(traj);

            // wait till we reach the last waypoint
            while(!reachedWaypoint(waypoints_subset.back())){
                ros::spinOnce();
            }
            ROS_INFO("Reached horizon at waypoint %d.", j-1);
        }

        goal_reached_ = true;

    }

    std::vector<Eigen::Vector3d> pathToWaypoints(quadrotor_common::TrajectoryPoint start_state, nav_msgs::Path plan)
    {
        int planlength = plan.poses.size();
        std::vector<Eigen::Vector3d> waypoints;
        for (int i = 0; i < planlength; i++)
        {
            Eigen::Vector3d waypoint;
            waypoint << plan.poses[i].pose.position.x, plan.poses[i].pose.position.y, start_state.position(2);
            waypoints.push_back(waypoint);
        }
        return waypoints;
    }
        
    void run()
    {
        ros::Rate command_rate(kExecLoopRate_);

        quadrotor_common::TrajectoryPoint start_state;
        start_state.position = autopilot_helper_.getCurrentReferenceState().position;
        start_state.heading = autopilot_helper_.getCurrentReferenceState().heading;

        while (goal_received_)
        {
            if (goal_reached_){
                goal_received_ = false;
                goal_reached_ = false;
                break;
            }

            ROS_INFO("Executing trajectory");
            nav_msgs::Path plan = getPlan(start_state, goal_pose_);
            rrt_traj_pub_.publish(plan);
            executeFullTrajectory(start_state, plan);

            autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::TRAJECTORY_CONTROL, 5.0, kExecLoopRate_);

            // Wait for autopilot to go back to hover
            autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::HOVER, 30.0, kExecLoopRate_);
            ros::Duration(3.0).sleep();

            ros::spinOnce();
            command_rate.sleep();
        }

        // Land
        // autopilot_helper_.sendLand();

        // // Wait for autopilot to go to land
        // autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::LAND, 10.0, kExecLoopRate_);

        // // Wait for autopilot to go to off
        // autopilot_helper_.waitForSpecificAutopilotState(autopilot::States::OFF, 10.0, kExecLoopRate_);

        // autopilot_helper_.sendOff();
    }

    void minSnapTrajPublisher(quadrotor_common::Trajectory& traj) {
        nav_msgs::Path path;
        for(auto point : traj.points){
            path.header.frame_id = "world";
            path.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.position(0);
            pose.pose.position.y = point.position(1);
            pose.pose.position.z = point.position(2);
            path.poses.push_back(pose);
        }
        snap_traj_pub_.publish(path);
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "local_planner");
    LocalPlanner planner;

    while(ros::ok()){
        planner.run();
        ros::spinOnce();
    }
    return 0;
}