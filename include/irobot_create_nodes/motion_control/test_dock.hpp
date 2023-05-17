#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include "capella_ros_service_interfaces/msg/charge_marker_visible.hpp"
#include "capella_ros_msg/msg/velocities.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/utils.h>
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include <unistd.h>
#include <queue>
#include <mutex>
#include <rclcpp_action/rclcpp_action.hpp>
#include "irobot_create_msgs/action/dock.hpp"
#include "angles/angles.h"
#include <thread>

using namespace std::chrono_literals;

struct robotPose
{
	double x;
	double y;
	double theta;
};

enum class DockStatus
{
	UNKNOW_ROBOT_POSE,
	SUCCESS,
	FAILURE
};

struct GoalRect
{
	int x_min;
	int x_max;
	int y_min;
	int y_max;
};

using Dock = irobot_create_msgs::action::Dock;
using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;

class TestDock : public rclcpp::Node
{
public:
	TestDock(std::string name, GoalRect goal_rect);

	bool get_robot_pose();
	double generate_random(int a, int b); // output: [a,b]/100.0
	void generate_goal_pose();
	void move_to_goal_pose();
	DockStatus start_docking();
	void run();

	rclcpp::Subscription<capella_ros_service_interfaces::msg::ChargeMarkerVisible>::SharedPtr dock_visible_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
	rclcpp::Subscription<capella_ros_msg::msg::Velocities>::SharedPtr raw_vel_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

	rclcpp::CallbackGroup::SharedPtr cb_group_sub_dock_visible_;
	rclcpp::CallbackGroup::SharedPtr cb_group_sub_robot_pose_;
	rclcpp::CallbackGroup::SharedPtr cb_group_sub_raw_vel_;

	std::shared_ptr<std::thread> __th_process_;

	void dock_visible_sub_callback(capella_ros_service_interfaces::msg::ChargeMarkerVisible msg);
	void raw_vel_sub_callback(capella_ros_msg::msg::Velocities msg);
	void robot_pose_sub_callback(geometry_msgs::msg::PoseStamped msg);
	void timer_pub_vel_callback();
	void dock_result_callback(const GoalHandleDock::WrappedResult &result);

	rclcpp_action::Client<Dock>::SharedPtr client_action_dock_;

	double bound_rotation(double);
	double bound_linear(double);

	bool sees_dock = false;
	bool success = false;
	robotPose robot_current_pose;
	robotPose robot_current_pose_sub;
	robotPose goal_pose;
	std::queue<capella_ros_msg::msg::Velocities> queue_raw_vel;
	int queue_raw_vel_size = 5;
	std::mutex queue_raw_vel_mutex;
	GoalRect goal_rect;

	int test_count = 5;
	int current_number = 0;
	int success_count = 0;
	int fail_count = 0;
	bool dock_success = false;
	bool dock_end = false;
	double time_start;
	double time_end;
	double time_max = 25.0; // seconds

	rclcpp::TimerBase::SharedPtr timer_pub_vel;
	geometry_msgs::msg::Twist pub_vel_msg;

	double max_rotation = 0.2;
	double min_rotation = 0.1;
	double max_linear = 0.1;
	double min_linear = 0.05;
	
	double time_undock = 5; // seconds

	DockStatus status_;
};