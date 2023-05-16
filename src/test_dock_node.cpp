#include <iostream>
#include <iomanip>
#include <irobot_create_nodes/motion_control/test_dock.hpp>

using namespace std;

int y_min = -70;
int y_max = 100;

int x_max = -70;
int x_min = -300;

size_t test_count = 100;



int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TestDock>("test_dock", x_min, x_max, y_min, y_max);
	rclcpp::executors::MultiThreadedExecutor executor;
    	executor.add_node(node);
	executor.spin();
    	rclcpp::shutdown();

	double goal_x, goal_y;
	for(size_t i = 0; i < test_count; i++)
	{
		// goal_x = generate_random(x_min, x_max);
		// goal_y = generate_random(y_min, y_max);
		cout << "x: "
		     << setw(5) << setfill('0') << setprecision(3) << left <<goal_x << ", " <<
		        "y: "
		     << setw(4) << setfill('0')  << setprecision(3) << left  << goal_y << endl;
	}

	getchar();
	return 0;
}
