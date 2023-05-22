#include <iostream>
#include <iomanip>
#include <irobot_create_nodes/motion_control/test_dock.hpp>

using namespace std;

int y_min = -70;  
int y_max = 100;

int x_max = -90; // > 0.32 + 0.1 * 5
int x_min = -200;

size_t test_count = 100;



int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	GoalRect goal_rect;
	goal_rect.x_min = x_min;
	goal_rect.x_max = x_max;
	goal_rect.y_min = y_min;
	goal_rect.y_max = y_max;
	auto node = std::make_shared<TestDock>("test_dock", goal_rect);
	rclcpp::executors::MultiThreadedExecutor executor;
    	executor.add_node(node);
	executor.spin();
    	rclcpp::shutdown();

	return 0;
}
