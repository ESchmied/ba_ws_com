#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

//#include "mpc_interfaces/msg/obstacle.hpp"
//#include "mpc_interfaces/msg/obstacle_array.hpp"

extern "C"
{
#include "grampc.h"
#include "time.h"
#include "my_cpp_py_pkg/userparam.h"
}

#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;

// In your header or class definition:
class MPCNode : public rclcpp::Node {
public:
    MPCNode() : Node("mpc_node") {
        RCLCPP_INFO(this->get_logger(), "MPCNode initialized");

        // Waypoint file
        waypoint_file_ = this->declare_parameter<std::string>("waypoint_file", "");
        user_param_.ref_length = this->declare_parameter<int>("considered_points", 0);

        // Topic parameters
        is_sim_ = this->declare_parameter<bool>("sim", true);
        fused_odom_ = nav_msgs::msg::Odometry();

        marker_frame_ = is_sim_ ? "map":"world";        

        odom_topic_ = this->declare_parameter<std::string>("odom", "");
        pose_topic_ = this->declare_parameter<std::string>("pose", "");
        twist_topic_ = this->declare_parameter<std::string>("twist", "");
        drive_topic_ = this->declare_parameter<std::string>("drive", "");

        // Declare all parameters
        user_param_.NX = this->declare_parameter<int>("NX", 0);
        user_param_.NU = this->declare_parameter<int>("NU", 0);
        user_param_.NH = this->declare_parameter<int>("NH", 0);

        user_param_.dt = this->declare_parameter<double>("dt", 0.0);
        user_param_.Nhor = this->declare_parameter<int>("Nhor", 0.0);
        user_param_.Thor = this->declare_parameter<double>("Thor", 0.0);

        user_param_.Q_pos = this->declare_parameter<double>("Q_pos", 0.0);
        user_param_.Q_theta = this->declare_parameter<double>("Q_theta", 0.0);
        user_param_.Q_vel = this->declare_parameter<double>("Q_vel", 0.0);

        user_param_.Q_pos_T = this->declare_parameter<double>("Q_pos_T", 0.0);
        user_param_.Q_theta_T = this->declare_parameter<double>("Q_theta_T", 0.0);
        user_param_.Q_vel_T = this->declare_parameter<double>("Q_vel_T", 0.0);

        user_param_.R_steer = this->declare_parameter<double>("R_steer", 0.0);
        user_param_.R_accel = this->declare_parameter<double>("R_accel", 0.0);

        user_param_.max_velocity = this->declare_parameter<double>("max_velocity", 0.0);
        user_param_.v_switch = this->declare_parameter<double>("v_switch", 0.0);

        user_param_.yaw_min = this->declare_parameter<double>("yaw_min", 0.0);
        user_param_.yaw_max = this->declare_parameter<double>("yaw_max", 0.0);
        user_param_.acc_min = this->declare_parameter<double>("acc_min", 0.0);
        user_param_.acc_max = this->declare_parameter<double>("acc_max", 0.0);

        user_param_.wheelbase = this->declare_parameter<double>("wheelbase", 0.0);
        user_param_.m = this->declare_parameter<double>("mass", 0.0);
        user_param_.lf = this->declare_parameter<double>("lf", 0.0);
        user_param_.lr = this->declare_parameter<double>("lr", 0.0);

        user_param_.C_af = this->declare_parameter<double>("C_af", 0.0);
        user_param_.C_ar = this->declare_parameter<double>("C_ar", 0.0);
        user_param_.I_z = this->declare_parameter<double>("I_z", 0.0);

        user_param_.centerline_dist = this->declare_parameter<double>("centerline_dist", 0.0);
        user_param_.soft_constraints = this->declare_parameter<bool>("soft_constraints", false);

        printf("soft-constraints: %d\n", user_param_.soft_constraints);

        // Load the reference path from CSV into a flat vector of doubles.
        flat_path_points_ = load_flat_pathpoints(
            waypoint_file_,
            1,   // Choose only every other point
            true // true if reversed, otherwise false
        );

        // Log the loaded waypoints and trajectory
        auto traj = convertPointsToTrajectory(flat_path_points_);

        RCLCPP_INFO(this->get_logger(), "Loaded %zu raw values(%zu waypoints)", flat_path_points_.size(), flat_path_points_.size() / 2);

        distance_boundary_ = computeDistanceBoundaries(flat_path_points_);

        // Subscribe to odometry.
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            10,
            std::bind(&MPCNode::odom_callback, this, std::placeholders::_1));

        if (!is_sim_) {
            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

            pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                pose_topic_,
                10,
                std::bind(&MPCNode::pose_callback, this, std::placeholders::_1));
            twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                twist_topic_,
                10,
                std::bind(&MPCNode::twist_callback, this, std::placeholders::_1));
        }

        /* // Subscribe to obstacles
        obstacle_subscriber_ = this->create_subscription<mpc_interfaces::msg::ObstacleArray>(
            "obstacle_data",
            10,
            std::bind(&MPCNode::obstacle_callback, this, std::placeholders::_1)
        );
 */

        // Other publishers…
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
        trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpc_trajectory", 10);
        active_ref_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mpc_ref_traj", 10);
        reference_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("reference_path", 10);
        nearest_point_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("nearest_point", 10);

        //opp_drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/opp_drive", 10);
        //obstacle_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);

        publish_reference_path(); // load in the reference path in RViz

        // Initialize GRAMPC
        // NOTE: ocp_dim in the .c file is only called once and hence needs to know the number of constraints 
        //user_param_.num_obstacles = this->declare_parameter<int>("max_num_obstacles", 0.0);
        init_grampc();
        //user_param_.num_obstacles = 0; // Reset to zero to wait for the actual obstacles via the topic

        grampc_printparam(grampc);
        grampc_printopt(grampc);
    }

    ~MPCNode(){
        auto stop_msg = ackermann_msgs::msg::AckermannDriveStamped();
        stop_msg.drive.speed = 0.0;
        if (drive_publisher_){
            drive_publisher_->publish(stop_msg);
        }
    }

private:
    // Publishers/subscribers...
    bool is_sim_;
    nav_msgs::msg::Odometry fused_odom_;

    std::string odom_topic_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    std::string pose_topic_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    std::string twist_topic_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;

    std::string drive_topic_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    //rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr opp_drive_publisher_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr active_ref_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr reference_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nearest_point_publisher_;
    //rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;

    //rclcpp::Subscription<mpc_interfaces::msg::ObstacleArray>::SharedPtr obstacle_subscriber_;

    // GRAMPC pointer
    TYPE_GRAMPC_POINTER(grampc)

    // Waypoint file
    std::string waypoint_file_;

    // Flat reference path (each waypoint stored as [x, y])
    vector<double> flat_path_points_;

    // This member holds the computed reference trajectory (flattened), i.e. for each prediction step: [x_ref, y_ref, yaw_ref]
    vector<double> ref_traj_;

    vector<double> distance_boundary_;

    std::string marker_frame_;

    // Store obstacle data
    std::vector<double> obstacle_sizes_;
    std::vector<double> obstacle_positions_;

    // Store the user parameters to pass to grampc
    UserParam user_param_;

    int infeasible_counter = 0;

    // --------------------------
    // Utility functions for handling the reference path/trajectory
    // --------------------------

    // Load CSV file into a flat vector of doubles.
    vector<double> load_flat_pathpoints(string waypoint_file, int d, bool reversed) {
        vector<double> points;
        ifstream file(waypoint_file);
        string line;

        // World -> Map transformation (only for real car needed), TODO: Find a better implementation, these values are for the currently used map
        double tx = -0.96517;
        double ty = -1.7104;
        double tz = 0.1992;

        double qx = 2.8928e-3;
        double qy = 1.3284e-2;
        double qz = -2.3513e-1;
        double qw = 9.7186e-1;

        double roll, pitch, yaw;
        tf2::Quaternion q(qx, qy, qz, qw);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "yaw: %f", yaw);

        int counter = 0;
        while (getline(file, line))
        {
            if (line.empty())
                continue;
            stringstream ss(line);
            string x_str, y_str;
            if (getline(ss, x_str, ',') && getline(ss, y_str, ','))
            {
                double x = stod(x_str);
                double y = stod(y_str);

                if (!is_sim_)
                {
                    double map_x = x;
                    double map_y = y;
                    x = tx + map_x * cos(yaw) - map_y * sin(yaw);
                    y = ty + map_x * sin(yaw) + map_y * cos(yaw);
                }

                if (counter % d != 0)
                {
                    counter++;
                    continue;
                }
                counter++;

                if (reversed)
                {
                    points.push_back(y);
                    points.push_back(x);
                }
                else
                {
                    points.push_back(x);
                    points.push_back(y);
                }
            }
        }

        if (reversed)
        {
            std::reverse(points.begin(), points.end());
        }
        // points.pop_back(); // In my_map_ref.csv the first and last point are the same causing some trouble later (division by zero leading to nan values)
        return points;
    }

    // Compute a reference trajetory based on the given points consting of N points starting at the closest one
    std::vector<double> computeReferenceTrajectory(const std::vector<double> &flat_points, int nearest_idx, int num_points_ahead) {
        int num_points = flat_points.size() / 2;
        std::vector<double> traj;

        for (int i = 0; i < num_points_ahead; ++i)
        {
            int idx = (nearest_idx + i) % num_points;

            double x_ref = flat_points[2 * idx];
            double y_ref = flat_points[2 * idx + 1];

            int next_idx = (idx + 1) % num_points;
            double x_next = flat_points[2 * next_idx];
            double y_next = flat_points[2 * next_idx + 1];

            double yaw_ref = atan2(y_next - y_ref, x_next - x_ref);

            traj.push_back(x_ref);
            traj.push_back(y_ref);
            traj.push_back(yaw_ref);

            // printf("idx: %d, next_idx: %d, xn: %f, xr: %f, yn: %f, yr: %f \n", idx, next_idx, x_next, x_ref, y_next, y_ref);
        }

        return traj;
    }

    // Compute an additional yaw reference for the whole path (currently, just used for debugging)
    vector<double> convertPointsToTrajectory(const vector<double> &flat_points) {
        int num_points = flat_points.size() / 2;
        vector<double> traj; // Will contain [x_ref, y_ref, yaw_ref] for each step.

        for (int i = 0; i < num_points; ++i)
        {
            double x_ref = flat_points[2 * i];
            double y_ref = flat_points[2 * i + 1];

            int next_idx = (i + 1) % num_points; // Loop around if last point

            double x_next = flat_points[2 * next_idx];
            double y_next = flat_points[2 * next_idx + 1];
            double yaw_ref = atan2(y_next - y_ref, x_next - x_ref);

            traj.push_back(x_ref);
            traj.push_back(y_ref);
            traj.push_back(yaw_ref);
        }
        return traj;
    }

    // Compute the inner and outer points that have the specified distance to the given path (currently, just used for visualization) 
    std::vector<double> computeDistanceBoundaries(const vector<double> &flat_points) {
        int num_points = flat_points.size() / 2;
        vector<double> boundary;

        for (int i = 0; i < num_points - 1; i++)
        {
            double current_x = flat_path_points_[2 * i];
            double current_y = flat_path_points_[2 * i + 1];

            double next_x = flat_path_points_[2 * (i + 1)];
            double next_y = flat_path_points_[2 * (i + 1) + 1];

            double x_dir = next_x - current_x;
            double y_dir = next_y - current_y;

            double length = sqrt((x_dir * x_dir) + (y_dir * y_dir));
            x_dir /= length;
            y_dir /= length;

            double x_dir_orth = -y_dir;
            double y_dir_orth = x_dir;

            boundary.push_back(current_x + x_dir_orth * user_param_.centerline_dist);
            boundary.push_back(current_y + y_dir_orth * user_param_.centerline_dist);
            boundary.push_back(current_x - x_dir_orth * user_param_.centerline_dist);
            boundary.push_back(current_y - y_dir_orth * user_param_.centerline_dist);
        }

        return boundary;
    }

    // --------------------------
    // Get the index of the closest path point to the current position
    typeInt getNearestIndex(double current_x, double current_y, const vector<double> &flat_points) {
        int num_points = flat_points.size() / 2;
        int nearest_idx = 0;
        double min_dist = numeric_limits<double>::max();
        for (int i = 0; i < num_points; i++)
        {
            double x = flat_points[2 * i];
            double y = flat_points[2 * i + 1];
            double d = sqrt((x - current_x) * (x - current_x) + (y - current_y) * (y - current_y));
            if (d < min_dist)
            {
                min_dist = d;
                nearest_idx = i;
            }
        }

        return nearest_idx;
    }

    // --------------------------
    // GRAMPC initialization (set parameters, dt, horizon, etc.)
    // --------------------------
    void init_grampc() {
        // Init grampc
        typeUSERPARAM *userparam = static_cast<void *>(&user_param_);
        grampc_init(&grampc, userparam);

        // Set initial state and control limits
        ctypeRNum x0[user_param_.NX] = {0.0};
        ctypeRNum umin[user_param_.NU] = {user_param_.yaw_min, user_param_.acc_min};
        ctypeRNum umax[user_param_.NU] = {user_param_.yaw_max, user_param_.acc_max};

        grampc_setparam_real_vector(grampc, "x0", x0);
        grampc_setparam_real_vector(grampc, "umin", umin);
        grampc_setparam_real_vector(grampc, "umax", umax);

        grampc_setparam_real(grampc, "dt", user_param_.dt);
        grampc_setparam_real(grampc, "t0", 0.0);

        grampc_setopt_int(grampc, "Nhor", user_param_.Nhor);
        grampc_setparam_real(grampc, "Thor", user_param_.Thor);

        // Important!! Without it the car drives serpentine-like
        grampc_setopt_string(grampc, "ShiftControl", "on");

        // Set number of gradient iterations (example)
        grampc_setopt_int(grampc, "MaxGradIter", 5); // 5
        grampc_setopt_int(grampc, "MaxMultIter", 10);  // 10

        grampc_setopt_real(grampc, "AugLagUpdateGradientRelTol", 1);

        grampc_setopt_string(grampc, "LineSearchType", "adaptive");

        // ctypeRNum ConstraintsAbsTol[1] = { 1e-2 };
        // grampc_setopt_real_vector(grampc, "ConstraintsAbsTol", ConstraintsAbsTol);
        grampc_setopt_string(grampc, "InequalityConstraints", "on");

        if (user_param_.soft_constraints)
        {
            grampc_setopt_string(grampc, "ConstraintsHandling", "extpen"); // Use soft-constraints
        }

        grampc_setopt_string(grampc, "ConvergenceCheck", "on");
        grampc_setopt_real(grampc, "ConvergenceGradientRelTol", 1e-6);

        // grampc_estim_penmin(grampc, 1);
    }

    // --------------------------
    // Callback functions for subscriptions
    // --------------------------
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        fused_odom_.pose.pose = msg->pose;

        odom_publisher_->publish(fused_odom_);
    }
    
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // The vicon system publishes the velocity in world coordinates
        // Combine the velocity in x and y direction for the magnitude
        double vx = msg->twist.linear.x;
        double vy = msg->twist.linear.y;

        fused_odom_.twist.twist.linear.x = sqrt(vx * vx + vy * vy);
        fused_odom_.twist.twist.angular.z = msg->twist.angular.z;
    }

    /* void obstacle_callback(const mpc_interfaces::msg::ObstacleArray::SharedPtr msg) {
        std::vector<mpc_interfaces::msg::Obstacle> obstacles = msg->obstacles;

        user_param_.num_obstacles = obstacles.size();
        user_param_.points_per_obstacle = 1;

        obstacle_sizes_.clear();
        obstacle_positions_.clear();

        for (int i = 0; i < obstacles.size(); i++) {
            obstacle_sizes_.push_back(obstacles[i].dimension);
            obstacle_positions_.push_back(obstacles[i].pose.position.x);
            obstacle_positions_.push_back(obstacles[i].pose.position.y);
        }
    }
 */
    // --------------------------
    // Main part happens in here
    // --------------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract state from odometry.
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        double v = msg->twist.twist.linear.x;
        double dyaw = msg->twist.twist.angular.z;

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        double roll, pitch, yaw;
        tf2::Quaternion q(qx, qy, qz, qw);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if (is_sim_)
        {
            // Move pose from rear axle to center of gravity
            // NOTE: It seems to work better when not shifting the pose,
            // x += user_param_.lr * cos(yaw);
            // y += user_param_.lr * sin(yaw);
        }

        RCLCPP_INFO(this->get_logger(), "----------");
        RCLCPP_INFO(this->get_logger(), "Odom received: x=%.6f, y=%.6f, yaw=%.2f, v=%.2f", x, y, yaw, v);

        // Compute the reference trajectory using the flat path points.
        int nearest_idx = getNearestIndex(x, y, flat_path_points_);
        auto ref_traj_ = computeReferenceTrajectory(flat_path_points_, nearest_idx, 0.8*flat_path_points_.size()/2); // just use 80% of the points each time to avoid looking behind

        // Update reference trajectory of user parameters
        user_param_.ref_traj = ref_traj_.data();

        /* // Update obstacle information. The variables are updated in the callback for the obstacle topic
        user_param_.obstacles_pos = obstacle_positions_.data();
        user_param_.obstacles_size= obstacle_sizes_.data();
 */
        grampc->userparam = static_cast<void *>(&user_param_);

        UserParam* param = (UserParam*)grampc->userparam;

        // Update current state.
        ctypeRNum x0[user_param_.NX] = {x, y, yaw, v, 0, dyaw}; // v:=vx, vy is not measured (TODO: Possible?)
        grampc_setparam_real_vector(grampc, "x0", x0);

        // Run GRAMPC.
        RCLCPP_INFO(this->get_logger(), "Starting GRAMPC run...");

        auto begin = std::chrono::steady_clock::now();
        grampc_run(grampc);
        auto end = std::chrono::steady_clock::now();
        auto calc_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        auto calc_time_mus = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        RCLCPP_INFO(this->get_logger(), "Calculation time: %d µs", calc_time_mus);

        RCLCPP_INFO(this->get_logger(), "Finished GRAMPC run. Status %d:", grampc->sol->status);
        grampc_printstatus(grampc->sol->status, STATUS_LEVEL_DEBUG);

        bool infeasible_flag = grampc->sol->status & 256; // 256 is the bitmask for STATUS_INFEASIBLE
        infeasible_counter *= infeasible_flag;
        infeasible_counter += infeasible_flag;

        bool infeasible = infeasible_counter > 100; // Only set if the flag was active for multiple runs

        // RCLCPP_INFO(this->get_logger(), "Elements: %d:", sizeof(grampc->sol->iter)/sizeof(grampc->sol->iter[0]));

        int mult_iters = sizeof(grampc->sol->iter)/sizeof(grampc->sol->iter[0]);
        RCLCPP_INFO(this->get_logger(), "MultIters (outer loop): %d", mult_iters);

        std::string grad_iters = "";
        for(int i=0; i<mult_iters; i++){
            grad_iters += std::to_string(grampc->sol->iter[i]);;

            if (i < mult_iters-1){
                grad_iters += ", ";
            }
        } 

        RCLCPP_INFO(this->get_logger(), "GradIters: %s", grad_iters.c_str());

        // Store state of next predicted step
        typeRNum last_next_state[user_param_.NX];
        for (int i = 0; i < user_param_.NX; i++)
        {
            last_next_state[i] = grampc->sol->xnext[i];
        }

        // Extract control command.
        double steering_angle = grampc->sol->unext[0]; // Extract the solution for k+1 from Grampc for the correct steering angle
        double acceleration = grampc->sol->unext[1];

        // TODO: At the beginning the velocity only changes slowly (can changing dt help?), maybe use grampc->rws->x to extract a step a bit more ahead 
        // double v_next = grampc->sol->xnext[3]; // Extract the velocity state of the next solution step
        double v_next = grampc->rws->x[2 * param->NX + 3]; // Extract the velocity state of the next solution step
        
        RCLCPP_INFO(this->get_logger(), "Published: Steering=%.2f, Speed=%.2f, Acceleration=%.2f", steering_angle, v_next, acceleration);

        // for (int i = 0; i < NHOR; ++i)
        // {
        //   double x_pred = grampc->rws->x[i * NX];
        //   double y_pred = grampc->rws->x[i * NX + 1];
        //   double yaw_pred = grampc->rws->x[i * NX + 2];
        //   double v_pred = grampc->rws->x[i * NX + 3];
        //   double dist = sqrt(POW(x_pred-x,2) + POW(y_pred-y,2));

        //   RCLCPP_INFO(this->get_logger(), "Step %d: x=%.3f, y=%.3f, yaw=%.2f, v=%.3f, dist=%.3f", i, x_pred, y_pred, yaw_pred, v_pred, dist);
        // }

        // Stopping criteria, modify if necessary
        bool command_is_nan = isnan(v_next) || isnan(steering_angle);
        bool high_computation_time = (calc_time_ms > 500);
        bool stop_car = infeasible || command_is_nan || high_computation_time;

        if (stop_car) {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            drive_msg.drive.steering_angle = 0.0;
            drive_publisher_->publish(drive_msg);

            /* auto opp_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            opp_drive_msg.drive.speed = 0.0;
            opp_drive_msg.drive.steering_angle = 0.0;
            opp_drive_publisher_->publish(opp_drive_msg); */

            RCLCPP_INFO(this->get_logger(), "Invalid MPC calculations. Stopping car and shutting down...");
            RCLCPP_INFO(this->get_logger(), "Last GRAMPC status %d. Infeasible flag: %s", grampc->sol->status, infeasible ? "yes":"no");
            RCLCPP_INFO(this->get_logger(), "Last drive command was NaN: %s", command_is_nan ? "yes":"no");
            RCLCPP_INFO(this->get_logger(), "Last computation time: %d ms", calc_time_ms);

            // Wait before shutdown, so that the last drive message still arrives
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
            rclcpp::shutdown();
            return;
        }

        // Publish control command.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = v_next;
        drive_msg.drive.steering_angle = steering_angle;
        drive_publisher_->publish(drive_msg);

        // Possibility to include publishing the opponents drive topic here
        // auto opp_drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // opp_drive_msg.drive.speed = 0.0;
        // opp_drive_msg.drive.steering_angle = 0.0;
        // opp_drive_publisher_->publish(opp_drive_msg);

        // Optionally publish predicted trajectory markers.
        publish_current_ref_trajectory();
        publish_mpc_trajectory();

        publish_reference_path();
        publish_distance_boundary();

        //publish_obstacles();
    }

    // --------------------------
    // Different Visualization Methods (TODO: Maybe clean up a bit)
    // --------------------------

    // Predicted MPC horizon trajectory
    void publish_mpc_trajectory() {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker point;
        point.header.frame_id = marker_frame_;
        point.header.stamp = this->now();
        point.ns = "mpc_horizon";
        point.type = visualization_msgs::msg::Marker::SPHERE;
        point.action = visualization_msgs::msg::Marker::ADD;
        // Scale and color of the sphere
        point.scale.x = 0.1;
        point.scale.y = 0.1;
        point.scale.z = 0.1;
        point.color.r = 1.0;
        point.color.g = 0.0;
        point.color.b = 0.0;
        point.color.a = 1.0;

        for (int i = 0; i < user_param_.Nhor; i++)
        {
            point.id = i;
            point.color.a = grampc->rws->x[i * user_param_.NX + 3] / user_param_.max_velocity;
            point.pose.position.x = grampc->rws->x[i * user_param_.NX];
            point.pose.position.y = grampc->rws->x[i * user_param_.NX + 1];
            point.pose.position.z = 0.1; // points are floating a bit over ground
            marker_array.markers.push_back(point);
        }
        trajectory_publisher_->publish(marker_array);
    }

    // Full reference path
    void publish_reference_path() {
        visualization_msgs::msg::Marker path;
        path.header.frame_id = marker_frame_;
        path.header.stamp = this->now();
        path.ns = "reference_path";
        path.id = 0;
        path.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path.action = visualization_msgs::msg::Marker::ADD;
        // scale and color of the line
        path.scale.x = 0.1;
        path.color.r = 0.0;
        path.color.g = 1.0;
        path.color.b = 0.0;
        path.color.a = 1.0;
        for (size_t i = 0; i < flat_path_points_.size() / 2; i++)
        {
            geometry_msgs::msg::Point p;
            p.x = flat_path_points_[2 * i];
            p.y = flat_path_points_[2 * i + 1];
            p.z = 0.1; // line is floating a bit over ground
            path.points.push_back(p);
        }
        reference_publisher_->publish(path);
    }

    // Currently considered reference points
    void publish_current_ref_trajectory() {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker point;
        point.header.frame_id = marker_frame_;
        point.header.stamp = this->now();
        point.ns = "mpc_ref_traj";
        point.type = visualization_msgs::msg::Marker::SPHERE;
        point.action = visualization_msgs::msg::Marker::ADD;
        // Scale and color of the sphere
        point.scale.x = 0.1;
        point.scale.y = 0.1;
        point.scale.z = 0.1;
        point.color.r = 0.0;
        point.color.g = 0.0;
        point.color.b = 1.0;
        point.color.a = 1.0;

        for (int i = 0; i < user_param_.ref_length; i++)
        {
            point.id = i;
            point.pose.position.x = user_param_.ref_traj[3 * i];
            point.pose.position.y = user_param_.ref_traj[3 * i + 1];
            point.pose.position.z = 0.1; // points are floating a bit over ground
            marker_array.markers.push_back(point);
        }
        active_ref_publisher_->publish(marker_array);
    }

    // Boundary from reference path according to defined distance
    void publish_distance_boundary() {
        visualization_msgs::msg::Marker path;
        path.header.frame_id = marker_frame_;
        path.header.stamp = this->now();
        path.ns = "distance_boundary_inner";
        path.id = 0;
        path.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path.action = visualization_msgs::msg::Marker::ADD;
        // scale and color of the line
        path.scale.x = 0.1;
        path.color.r = 0.0;
        path.color.g = 1.0;
        path.color.b = 1.0;
        path.color.a = 1.0;
        for (size_t i = 0; i < distance_boundary_.size() / 4; i++)
        {
            geometry_msgs::msg::Point p;
            p.x = distance_boundary_[4 * i];
            p.y = distance_boundary_[4 * i + 1];
            p.z = 0.1; // line is floating a bit over ground
            path.points.push_back(p);
        }
        reference_publisher_->publish(path);

        path.points.clear();

        path.header.frame_id = marker_frame_;
        path.header.stamp = this->now();
        path.ns = "distance_boundary_outer";
        path.id = 0;
        path.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path.action = visualization_msgs::msg::Marker::ADD;
        // scale and color of the line
        path.scale.x = 0.1;
        path.color.r = 0.0;
        path.color.g = 1.0;
        path.color.b = 1.0;
        path.color.a = 1.0;
        for (size_t i = 0; i < distance_boundary_.size() / 4; i++)
        {
            geometry_msgs::msg::Point p;
            p.x = distance_boundary_[4 * i + 2];
            p.y = distance_boundary_[4 * i + 3];
            p.z = 0.1; // line is floating a bit over ground
            path.points.push_back(p);
        }
        reference_publisher_->publish(path);
    }

    /* // Obstacles
    void publish_obstacles(){
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker point;
        point.header.frame_id = marker_frame_;
        point.header.stamp = this->now();
        point.ns = "obstacles";
        point.type = visualization_msgs::msg::Marker::SPHERE;
        point.action = visualization_msgs::msg::Marker::ADD;
        // Scale and color of the sphere
        point.scale.z = 0.1;
        point.color.r = 1.0;
        point.color.g = 0.0;
        point.color.b = 0.0;
        point.color.a = 1.0;

        for (int i = 0; i < user_param_.num_obstacles; i++) {
            point.id = i;
            point.scale.x = 2 * user_param_.obstacles_size[i];
            point.scale.y = 2 * user_param_.obstacles_size[i];
            point.pose.position.x = user_param_.obstacles_pos[2 * i * user_param_.points_per_obstacle];
            point.pose.position.y = user_param_.obstacles_pos[2 * i * user_param_.points_per_obstacle + 1];
            point.pose.position.z = 0.0;

            marker_array.markers.push_back(point);
        }

        obstacle_publisher_->publish(marker_array);

    }*/
}; 

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::cout << "Start \n";
    rclcpp::spin(std::make_shared<MPCNode>());
    std::cout << "Quit \n";

    rclcpp::shutdown();
    return 0;
}
