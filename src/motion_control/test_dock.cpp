#include "irobot_create_nodes/motion_control/test_dock.hpp"

using namespace std;
using namespace std::placeholders;

TestDock::TestDock(std::string name, GoalRect goal_rect) : Node(name)
{
        client_action_dock_ = rclcpp_action::create_client<Dock>(this, "/dock");

        cb_group_sub_dock_visible_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt_dock_visible = rclcpp::SubscriptionOptions();
        sub_opt_dock_visible.callback_group = cb_group_sub_dock_visible_;
        dock_visible_sub_ = this->create_subscription<capella_ros_service_interfaces::msg::ChargeMarkerVisible>("/marker_visible", 5, std::bind(&TestDock::dock_visible_sub_callback, this, _1), sub_opt_dock_visible);
        
        cb_group_sub_raw_vel_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt_raw_vel = rclcpp::SubscriptionOptions();
        sub_opt_raw_vel.callback_group = cb_group_sub_raw_vel_;
        raw_vel_sub_ = this->create_subscription<capella_ros_msg::msg::Velocities>("/raw_vel", 5, std::bind(&TestDock::raw_vel_sub_callback, this, _1), sub_opt_raw_vel);
        
        cb_group_sub_robot_pose_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt_robot_pose = rclcpp::SubscriptionOptions();
        sub_opt_robot_pose.callback_group = cb_group_sub_robot_pose_;
        robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/aruco_single/pose", 5, std::bind(&TestDock::robot_pose_sub_callback, this, _1), sub_opt_robot_pose);
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // --ros-args -p test_count:=10
        test_count = this->get_parameter_or("test_count", 6);
        cout << "test_count: " << test_count << endl;
        this->goal_rect = goal_rect;

        __th_process_ = std::make_shared<std::thread>(std::bind(&TestDock::run, this));

}

bool TestDock::get_robot_pose()
{
        time_start = this->now().seconds();
        RCLCPP_INFO(this->get_logger(), "********** get robot_pose **********");
        if (sees_dock)
        {
                // cout << "sees_dock 1" << endl;
                sleep(2);
                robot_current_pose = robot_current_pose_sub;
                RCLCPP_INFO(this->get_logger(), "robot pose => x: %f, y: %f", robot_current_pose.x, robot_current_pose.y);
                return true;
        }
        else
        {
                double rotation_angle = 0.;
                this->pub_vel_msg = geometry_msgs::msg::Twist();
                this->pub_vel_msg.angular.z = 0.2;
                timer_pub_vel = this->create_wall_timer(100ms, std::bind(&TestDock::timer_pub_vel_callback, this));
                double pre_time, now_time, dt;
                pre_time = this->now().seconds();
                double angular_z;
                while (std::abs(rotation_angle) < M_PI && !sees_dock)
                {
                        now_time = this->now().seconds();
                        if (now_time - time_start > time_max)
                        {
                                status_ = DockStatus::UNKNOW_ROBOT_POSE;
                                return false;
                        }
                        dt = now_time - pre_time;
                        pre_time = now_time;
                        {
                                const std::lock_guard<std::mutex> lock(queue_raw_vel_mutex);
                                if (queue_raw_vel.size() == 0)
                                {
                                        angular_z = 0;
                                }
                                else
                                {
                                        angular_z = queue_raw_vel.front().angular_z;
                                        queue_raw_vel.pop();
                                }
                                rotation_angle += dt * angular_z;
                        }
                        usleep(30 * 1000);
                }
                if(sees_dock)
                {
                        sleep(2);
                        robot_current_pose = robot_current_pose_sub;
                        RCLCPP_INFO(this->get_logger(), "robot pose => x: %f, y: %f", robot_current_pose.x, robot_current_pose.y);
                }
                timer_pub_vel.reset();
                return (sees_dock ? true : false);
        }
}

// https://blog.csdn.net/onion23/article/details/118558454
double TestDock::generate_random(int a, int b)
{
        return ((rand() % (b - a + 1)) + a) / 100.0;
}

void TestDock::generate_goal_pose()
{        
        RCLCPP_INFO(this->get_logger(), "********** generate goal pose **********");
        goal_pose.x = generate_random(goal_rect.x_min, goal_rect.x_max);
        goal_pose.y = generate_random(goal_rect.y_min, goal_rect.y_max);
        RCLCPP_INFO(this->get_logger(), "goal pose => x: %5.2f, y: %5.2f", goal_pose.x, goal_pose.y);
}

void TestDock::move_to_goal_pose()
{
        RCLCPP_INFO(this->get_logger(), "********** move to goal pose **********");
        double dist_yaw, dist_linear;
        double dt_y, dt_x;
        dt_y = goal_pose.y - robot_current_pose.y;
        dt_x = goal_pose.x - robot_current_pose.x;
        dist_yaw = angles::shortest_angular_distance(robot_current_pose.theta,
                                                     std::atan2(dt_y, dt_x));
        dist_linear = std::hypot(dt_y, dt_x);

        // cout << "robot pose => x: " << robot_current_pose.x << ", y: " << robot_current_pose.y << endl;
        // cout << "robot theta: " << robot_current_pose.theta << endl;
        // cout << "goal pose  => x: " << goal_pose.x << ", y: " << goal_pose.y << endl;
        // cout << "goal theta: " << std::atan2(dt_y, dt_x) << endl;
        // cout << "dist yaw: " << dist_yaw << endl;

        // angle to goal;
        this->pub_vel_msg = geometry_msgs::msg::Twist();
        this->pub_vel_msg.angular.z = bound_rotation(dist_yaw);
        timer_pub_vel = this->create_wall_timer(100ms, std::bind(&TestDock::timer_pub_vel_callback, this));
        double pre_time = this->get_clock()->now().seconds();
        double now_time, dt;
        while (std::abs(dist_yaw) > 0.05)
        {
                {
                        const std::lock_guard<std::mutex> lock(queue_raw_vel_mutex);
                        now_time = this->get_clock()->now().seconds();
                        dt = now_time - pre_time;
                        pre_time = now_time;
                        double angular_z = 0.;
                        if (queue_raw_vel.size() > 0)
                        {
                                angular_z = queue_raw_vel.front().angular_z;
                                queue_raw_vel.pop();
                        }
                        dist_yaw -= dt * angular_z;
                        // cout << "queue size: " << queue_raw_vel.size() << endl;
                        // cout << "dt: " << dt << endl;
                        // cout << "angular_z: " << angular_z << endl;
                        // cout << "dist yaw: " << dist_yaw << endl;
                }
                usleep(30 * 1000);
        }
        timer_pub_vel.reset();

        // move to goal;
        this->pub_vel_msg = geometry_msgs::msg::Twist();
        this->pub_vel_msg.linear.x = -bound_linear(dist_linear);
        timer_pub_vel = this->create_wall_timer(100ms, std::bind(&TestDock::timer_pub_vel_callback, this));
        while (std::abs(dist_linear) > 0.05)
        {
                {
                        const std::lock_guard<std::mutex> lock(queue_raw_vel_mutex);
                        now_time = this->get_clock()->now().seconds();
                        dt = now_time - pre_time;
                        pre_time = now_time;
                        double linear_x = 0;
                        if (queue_raw_vel.size() > 0)
                        {
                                linear_x = queue_raw_vel.front().linear_x;
                                queue_raw_vel.pop();
                        }
                        dist_linear += dt * linear_x;
                        // cout << "dt: " << dt << endl;
                        // cout << "linear_x: " << linear_x << endl;
                        // cout << "dist_linear: " << dist_linear << endl;
                }
                usleep(30 * 1000);
        }
        timer_pub_vel.reset();
}

DockStatus TestDock::start_docking()
{
        dock_end = false;

        if(dock_success)
        {
                RCLCPP_INFO(this->get_logger(), "********** undock **********");
                this->pub_vel_msg = geometry_msgs::msg::Twist();
                this->pub_vel_msg.linear.x = 0.10;
                timer_pub_vel = this->create_wall_timer(100ms, std::bind(&TestDock::timer_pub_vel_callback, this));
                double now_time, pre_time, dt;
                pre_time = this->now().seconds();
                double time_elapsed = time_undock;
                while(time_elapsed > 0)
                {
                        now_time = this->now().seconds();
                        dt = now_time - pre_time;
                        pre_time = now_time;
                        time_elapsed -= dt;
                        usleep(100* 1000);
                }
                timer_pub_vel.reset();
                dock_success = false;
        }

        if (!get_robot_pose())
        {
                status_ = DockStatus::UNKNOW_ROBOT_POSE;
                return status_;
        }

        generate_goal_pose();
        
        move_to_goal_pose();

        // send goal
        RCLCPP_INFO(this->get_logger(), "********** Dock action **********");
        while (!client_action_dock_->wait_for_action_server(1s))
        {
                RCLCPP_ERROR(this->get_logger(), "Dock service not online, waiting.");
        }
        auto goal_msg = Dock::Goal();
        auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&TestDock::dock_result_callback, this, _1);

        this->client_action_dock_->async_send_goal(goal_msg, send_goal_options);

        while (!dock_end)
        {
                sleep(1);
        }
        return status_;
}

void TestDock::run()
{
        current_number = 0;
        for (; current_number < test_count; current_number++)
        {
                RCLCPP_INFO(this->get_logger(), "---------------- Number %3d ---------------- ", current_number);
                DockStatus result = start_docking();
                switch (result)
                {
                case DockStatus::UNKNOW_ROBOT_POSE: 
                {
                        RCLCPP_INFO(this->get_logger(), "UNKNOW_ROBOT_POSE");
                        break;
                } 
                case DockStatus::SUCCESS:
                {
                        RCLCPP_INFO(this->get_logger(), "SUCCESS");
                        break;
                } 
                case DockStatus::FAILURE: 
                {
                        RCLCPP_INFO(this->get_logger(), "FAILURE");
                        break;
                } 
                default:
                        break;
                }
                cout << endl; // SPACE
                sleep(5);
        }

        double success_rate = success_count * 100 / (float)test_count;
        RCLCPP_INFO(this->get_logger(), "Test count: %d, success: %d, fail: %d, success rate: %.2f%%", test_count, success_count, fail_count, success_rate);
        RCLCPP_INFO(this->get_logger(), "********************************************************************************");
}

double TestDock::bound_rotation(double z)
{
        if (std::abs(z) > max_rotation)
        {
                return std::copysign(max_rotation, z);
        }
        else if (std::abs(z) < min_rotation)
        {
                return std::copysign(min_rotation, z);
        }
        else
        {
                return z;
        }
}

double TestDock::bound_linear(double x)
{
        if (std::abs(x) > max_linear)
        {
                return std::copysign(max_linear, x);
        }
        else if (std::abs(x) < min_linear)
        {
                return std::copysign(min_linear, x);
        }
        else
        {
                return x;
        }
}

void TestDock::dock_result_callback(const GoalHandleDock::WrappedResult &result)
{
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
                success_count++;
                dock_success = true;
                status_ = DockStatus::SUCCESS;
                break;
        case rclcpp_action::ResultCode::ABORTED:
        default:
                fail_count++;
                status_ = DockStatus::FAILURE;
                break;
        }
        dock_end = true;
}

void TestDock::dock_visible_sub_callback(capella_ros_service_interfaces::msg::ChargeMarkerVisible msg)
{
        // cout << "charge visible callback" << endl;
        sees_dock = msg.marker_visible;
}

void TestDock::raw_vel_sub_callback(capella_ros_msg::msg::Velocities msg)
{
        // cout << "raw_vel_sub_callback" << endl;
        const std::lock_guard<std::mutex> lock(queue_raw_vel_mutex);
        if ((int)queue_raw_vel.size() < queue_raw_vel_size)
        {
                queue_raw_vel.emplace(msg);
                // cout << "emplace" << endl;
        }
        else
        {
                queue_raw_vel.pop();
                queue_raw_vel.emplace(msg);
        }
}

void TestDock::robot_pose_sub_callback(geometry_msgs::msg::PoseStamped msg)
{
        // cout << "robot pose callback " << endl;
        tf2::Transform transform;
        tf2::convert(msg.pose, transform);
        robot_current_pose_sub.x = transform.getOrigin().getX();
        robot_current_pose_sub.y = transform.getOrigin().getY();
        robot_current_pose_sub.theta = tf2::getYaw(transform.getRotation());
}

void TestDock::timer_pub_vel_callback()
{
        cmd_vel_pub_->publish(this->pub_vel_msg);
}