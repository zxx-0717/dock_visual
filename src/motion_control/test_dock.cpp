#include "irobot_create_nodes/motion_control/test_dock.hpp"

using namespace std;
using namespace std::placeholders;

TestDock::TestDock(std::string name, int x_min, int x_max, int y_min, int y_max, int test_count) : Node(name),
x_min(x_min), x_max(x_max), y_min(y_min), y_max(y_max), test_count(test_count)
{
        client_dock_ = create_client<std_srvs::srv::Empty>("charger/start_docking");
        // dock_visible_sub_ = this->create_subscription("/marker_visible", 5, std::bind(&TestDock::dock_visible_sub_callback, this, _1));
        goal_array = new robotPose[test_count];
}

// https://blog.csdn.net/onion23/article/details/118558454
double TestDock::generate_random(int a, int b)
{
        return ((rand() % (b - a + 1)) + a);
}

void TestDock::generate_goal_pose()
{
    goal_pose.x = generate_random(x_min, x_max);
    goal_pose.y = generate_random(y_min, y_max);    
}

void TestDock::dock_visible_sub_callback(capella_ros_service_interfaces::msg::ChargeMarkerVisible msg)
{
        sees_dock = msg.marker_visible;
}

void TestDock::raw_vel_sub_callback(capella_ros_msg::msg::Velocities msg)
{
        raw_vel = msg;
}





