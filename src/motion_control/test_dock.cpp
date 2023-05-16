#include "irobot_create_nodes/motion_control/test_dock.hpp"

using namespace std;
using namespace std::placeholders;

TestDock::TestDock(std::string name, GoalRect goal_rect) : Node(name)
{
        client_action_dock_ = rclcpp_aciton::create_client<Dock>(this, "/dock");
        dock_visible_sub_ = this->create_subscription("/marker_visible", 5, std::bind(&TestDock::dock_visible_sub_callback, this, _1));
        raw_vel_sub_ = this->create_subscription("/raw_vel", 5, std::bind(&TestDock::raw_vel_sub_callback, this, _1));
        robot_pose_sub = this->create_subscription("/aruco/single_pose", 5, std::bind(&TestDock::robot_pose_sub_callback, this, _1));
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // --ros-args -p test_count:=10
        test_count = this->get_parameter_or("test_count", 5);
}

bool TestDock::get_robot_pose()
{
        time_start = this->now().seconds();
        if (sees_dock)
        {
                robot_current_pose = robot_current_pose_sub;
                retrun true;
        }
        else
        {
                double rotation_angle = 0;
                timer_pub_vel = this->create_wall_timer(100ms, std::bind(&TestDock::timer_pub_vel_callback, this, 0, 0.2));
                double pre_time, now_time, dt;
                pre_time = this->now().seconds();
                double angular_z;
                while (std::abs(rotation_angle) < M_PI && !sees_dock)
                {
                        now_time = this->now().seconds();
                        if (now_time - time_start > time_max)
                        {
                                return false;
                        }
                        dt = now_time - pre_time;
                        pre_time = now_time;
                        const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
                        if (queue_raw_vel.size() == 0)
                        {
                                angular_z = 0;
                        }
                        else
                        {
                                angular_z = queue_raw_vel.front();
                                queue_raw_vel.pop();
                        }
                        rotation_angle += dt * angular_z;
                        usleep(30 * 1000);
                }
                timer_pub_vel.reset();
                return (sees_dock ? true : false);
        }
}

// https://blog.csdn.net/onion23/article/details/118558454
double TestDock::generate_random(int a, int b)
{
        return ((rand() % (b - a + 1)) + a);
}

void TestDock::generate_goal_pose()
{
        goal_pose.x = generate_random(goal_rect.x_min, goal_rect.x_max);
        goal_pose.y = generate_random(goal_rect.y_min, goal_rect.y_max);
}

void TestDock::dock_visible_sub_callback(capella_ros_service_interfaces::msg::ChargeMarkerVisible msg)
{
        sees_dock = msg.marker_visible;
}

void TestDock::raw_vel_sub_callback(capella_ros_msgs::msg::Velocities msg)
{
        const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
        if (queue_raw_vel_size.size() < queue_raw_vel_size)
        {
                queue_raw_vel_size.emplace_back(msg);
        }
        else
        {
                queue_raw_vel.pop();
                queue_raw_vel_size.emplace_back(msg);
        }
}

void TestDock::robot_pose_sub_callback(geometry_msgs::msg::PoseStamped msg)
{
        tf2::Transform transform;
        tf2::convert(msg->pose, transform);
        robot_current_pose_sub.x = transfrom.getOrigin().getX();
        robot_current_pose_sub.y = transform.getOrigin().getY();
        robot_current_pose_sub.theta = tf2::getYaw(transform.getRotation());
}

void TestDock::timer_pub_vel_callback(double linear_x, double angular_z)
{
        geometry_msgs::msg::Twist msg;
        msg.liner_x = linear_x;
        msg.angular_z = angular_z;
        cmd_vel_pub_->publish(msg);
}

void TestDock::move_to_goal_pose()
{
        double dist_yaw, dist_linear;
        double linear_x, angular_z;
        dist_yaw = angles::shortest_angular_distance(robot_current_pose.theta,
                                                     std::atan2(goal_pose.y - robot_current_pose.y, goal_pose.x - robot_pose.x));
        dist_linear = std::hypot(goal_pose.y - robot_current_pose.y, goal_pose.x - robot_pose.x);

        // angle to goal;
        timer_pub_vel = this->create_wall_timer(100ms, std::bind(&TestDock::timer_pub_vel_callback, this, 0, bound_rotation(dist_yaw)));
        double pre_time = this->get_clock()->now().seconds();
        double now_time, dt;
        while (std::abs(dist_yaw) > 0.05)
        {
                const std::lock_guard<std::mutex> lock(queue_raw_vel_mutex);
                now_time = this->get_clock()->now().senconds();
                dt = now_time - pre_time;
                pre_time = now_time;
                double angular_z = 0.;
                if (queue_raw_vel.size() > 0)
                {
                        angular_z = queue_raw_vel.front().angular_z;
                }
                dist_yaw += dt * angular_z;
                usleep(30 * 1000);
        }
        timer_pub_vel.reset();

        // move to goal;
        timer_pub_vel = this->create_wall_timer(100ms, std::bind(&TestDock::timer_pub_vel_callback, this, bound_linear(dist_linear), 0));
        while (std::abs(dist_linear) > 0.05)
        {
                const std::lock_guard<std::mutex> lock(queue_raw_vel_mutex);
                now_time = this->get_clock()->now().senconds();
                dt = now_time - pre_time;
                pre_time = now_time;
                double linear_x;
                if (queue_raw_vel.size() > 0)
                {
                        linear_x = queue_raw_vel.front().linear_x;
                }
                dist_yaw += dt * linear_x;
                usleep(30 * 1000);
        }
        timer_pub_vel.reset();
}

DockStatus TestDock::start_docking()
{
        dock_end = false;
        if (!get_robot_pose())
        {
                return DockStatus::UNKNOW_ROBOT_POSE;
        }

        generate_goal_pose();
        move_to_goal_pose();

        // send goal
        while (!client_action_dock_.wait_for_action_server(1s))
        {
                RCLCPP_ERROR(this->get_logger(), "Dock service not online, waiting.");
        }
        auto goal_msg = Dock::Goal();
        auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&TestDock::dock_result_callback, this, _1);

        this->client_action_dock->async_send_goal(goal_msg, send_goal_options);

        while (!dock_end)
        {
                sleep(1);
        }

        RCLCPP_INFO(this->get_logger(), "------------------------------ Number %3d ------------------------------", current_number);
        RCLCPP_INFO(this->get_logger(), "goal pose => x: %5.2f, y: %5.2f", goal_pose.x, goal_pose.y);
}

void TestDock::dock_result_callback(const GoalHandleDock::WrappedResult &result)
{
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
                success_count++;
                break;
        case rclcpp_action::ResultCode::ABORTED:
        default:
                fail_count++;
                break;
        }
        dock_end = true;
}

void TestDock::run()
{
        current_number = 0;
        for (; current_number < test_count; current_number++)
        {
                DockStatus result = start_docking();
                switch (result)
                {
                case DockStatus::UNKNOW_ROBOT_POSE {
                        RCLCPP_INFO(this->get_logger(), "UNKNOW_ROBOT_POSE");
                        break;
                } case DockStatus::SUCCESS {
                        RCLCPP_INFO(this->get_logger(), "SUCCESS");
                        break;
                } case DockStatus::FAILURE {
                        RCLCPP_INFO(this->get_logger(), "FAILURE");
                        break;
                } default:
                        break;
                }
        }

        RCLCPP_INFO(this->get_logger(), "********************************************************************************");
        double success_rate = success_rate * 100 / (float)test_count;
        RCLCPP_INFO(this->get_logger(), "Test count: %d, success: %d, fail: %d, success rate: %.2f%%", success_count, fail_count, success_rate)
        RCLCPP_INFO(this->get_logger(), "********************************************************************************");
}

void TestDock::bound_rotation(double &z)
{
        if (z > max_rotation)
        {
                z = max_rotation;
        }
        else if (z < min_rotation)
        {
                z = min_rotation;
        }
}

void TestDock::bound_linear(double &x)
{
        if (x > max_linear)
        {
                x = max_linear;
        }
        else if (x < min_linear)
        {
                x = min_linear;
        }
}
