#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include "capella_ros_service_interfaces/msg/charge_marker_visible.hpp"
#include "capella_ros_msg/msg/velocities.hpp"
#include "std_srvs/srv/empty.hpp"

struct robotPose
{
	double x;
	double y;
	double theta;
};

class TestDock : public rclcpp::Node
{
public:
TestDock(std::string name, int x_min, int x_max, int y_min, int y_max, int test_count);

double generate_random(int a, int b); // output: [a,b]/100.0
void generate_goal_pose();

void move_to_goal_pose();

void start_docking();

rclcpp::Subscription<capella_ros_service_interfaces::msg::ChargeMarkerVisible>::SharedPtr dock_visible_sub_;
rclcpp::Subscription<capella_ros_msg::msg::Velocities>::SharedPtr raw_vel_sub_;

void dock_visible_sub_callback(capella_ros_service_interfaces::msg::ChargeMarkerVisible msg);
void raw_vel_sub_callback(capella_ros_msg::msg::Velocities msg);

rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_dock_;

bool sees_dock = false;
bool success = false;
robotPose robot_pose;
robotPose goal_pose;
capella_ros_msg::msg::Velocities raw_vel;
const int x_min, x_max, y_min, y_max;
const int test_count;
robotPose * goal_array;
};