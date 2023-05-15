// Copyright 2021 iRobot Corporation. All Rights Reserved.
// @author Justin Kearns (jkearns@irobot.com)

#ifndef IROBOT_CREATE_NODES__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_
#define IROBOT_CREATE_NODES__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_

#include <deque>
#include <functional>
#include <mutex>
#include <vector>

#include "angles/angles.h"
#include "boost/optional.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_nodes/motion_control/behaviors_scheduler.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <time.h>
#include <rclcpp/time.hpp>
#include <capella_ros_msg/msg/velocities.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace std;

namespace irobot_create_nodes
{

/**
 * @brief This class provides an API to give velocity commands given a goal and position.
 */
class SimpleGoalController
{
public:
SimpleGoalController()
{
}

/// \brief Structure to keep information for each point in commanded path
//  including pose with position and orientation of point
//  radius that is considered close enough to achieving the point
//  drive_backwards whether the robot should drive backwards towards the point (for undocking)
struct CmdPathPoint
{
	CmdPathPoint(tf2::Transform p, float r, bool db)
		: pose(p), radius(r), drive_backwards(db) {
	}
	tf2::Transform pose;
	float radius;
	bool drive_backwards;
};
using CmdPath = std::vector<CmdPathPoint>;

/// \brief Set goal path for controller along with max rotation and translation speed
void initialize_goal(const CmdPath & cmd_path, double max_rotation, double max_translation)
{
	const std::lock_guard<std::mutex> lock(mutex_);
	// Convert path points to goal points
	goal_points_.clear();
	goal_points_.resize(cmd_path.size());
	for (size_t i = 0; i < cmd_path.size(); ++i) {
		GoalPoint & gp = goal_points_[i];
		const tf2::Vector3 & pt_position = cmd_path[i].pose.getOrigin();
		gp.x = pt_position.getX();
		gp.y = pt_position.getY();
		gp.theta = tf2::getYaw(cmd_path[i].pose.getRotation());
		gp.radius = cmd_path[i].radius;
		gp.drive_backwards = cmd_path[i].drive_backwards;
	}
	navigate_state_ = NavigateStates::LOOKUP_ARUCO_MARKER;
	max_rotation_ = max_rotation;
	max_translation_ = max_translation;
}

/// \brief Clear goal
void reset()
{
	const std::lock_guard<std::mutex> lock(mutex_);
	goal_points_.clear();
}

// \brief Generate velocity based on current position and next goal point looking for convergence
// with goal point based on radius.
// \return empty optional if no goal or velocity command to get to next goal point
BehaviorsScheduler::optional_output_t get_velocity_for_position(
	const tf2::Transform & current_pose, bool sees_dock, bool is_docked,
	capella_ros_msg::msg::Velocities raw_vel_msg, rclcpp::Clock::SharedPtr clock_)
{
	time_start = std::chrono::high_resolution_clock::now();
	BehaviorsScheduler::optional_output_t servo_vel;
	const std::lock_guard<std::mutex> lock(mutex_);
	if (is_docked)
	{
		std::cout << "*************** is docked *************" << std::endl << std::endl;
		goal_points_.clear();
		first_sees_dock = true;
	}
	if (goal_points_.size() == 0) {
		return servo_vel;
	}

	double current_angle;
	tf2::Vector3 current_position;
	if (navigate_state_ >= NavigateStates::ANGLE_TO_GOAL)
	{
		current_angle = tf2::getYaw(current_pose.getRotation());
		current_position = current_pose.getOrigin();
	}

	// Generate velocity based on current position and next goal point looking for convergence
	// with goal point based on radius.
	switch (navigate_state_) {
	case NavigateStates::LOOKUP_ARUCO_MARKER:
	{
		std::cout << "------------- LOOKUP_ARUCO_MARKER -------------\n";
		servo_vel = geometry_msgs::msg::Twist();
		if (sees_dock)
		{
			if(first_sees_dock)
			{
				first_sees_dock = false;
				first_sees_dock_time = clock_->now().seconds();
			}
			now_time = clock_->now().seconds();
			if (now_time - first_sees_dock_time > 2.0)
			{
				double robot_x = current_pose.getOrigin().getX();
				double robot_y = current_pose.getOrigin().getY();
				double theta = std::atan2(std::abs(robot_y), std::abs(robot_x - 0.92));
				cout << "robot_x: " << robot_x << endl;
				cout << "theta: " << theta << endl;
				cout << "thr_angle_diff: " << thre_angle_diff << endl;
				if (theta < thre_angle_diff && std::abs(robot_x) > (0.32 + 0.1 + 0.7)) // 0.7 <= 0.5 + 0.2(x_error)
				{
					navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
				}
				else
				{
					dist_buffer_point = std::hypot(robot_x - buffer_goal_point_x, robot_y - buffer_goal_point_y);
					robot_current_yaw = tf2::getYaw(current_pose.getRotation());
					robot_angle_to_buffer_point_yaw = std::atan2(buffer_goal_point_y - robot_y, buffer_goal_point_x - robot_x);
					dist_buffer_point_yaw = angles::shortest_angular_distance(robot_current_yaw,
					                                                          robot_angle_to_buffer_point_yaw);
					cout << "dist_buffer_point: " << dist_buffer_point << endl;
					cout << "dist_buffer_point_yaw: " << dist_buffer_point_yaw << endl;
					pre_time = clock_->now().seconds();
					navigate_state_ = NavigateStates::ANGLE_TO_BUFFER_POINT;
				}


			}
		}
		else
		{
			servo_vel->angular.z = 0.2;
		}
		break;
	}
	case NavigateStates::ANGLE_TO_BUFFER_POINT:
	{
		std::cout << "------------- ANGLE_TO_BUFFER_POINT -------------\n";
		servo_vel = geometry_msgs::msg::Twist();
		now_time = clock_->now().seconds();
		double dt = now_time - pre_time;
		// cout << "now_time: " << now_time << endl;
		// cout << "pre_time: " << pre_time << endl;
		cout << "dt: " << dt << endl;
		cout << "raw_angular.z: " << raw_vel_msg.angular_z << endl;
		cout << "delta_angular: " << raw_vel_msg.angular_z * dt << endl;
		cout << "dist_buffer_point_yaw pre: " << dist_buffer_point_yaw << endl;
		robot_current_yaw += raw_vel_msg.angular_z * dt;
		dist_buffer_point_yaw = angles::shortest_angular_distance(robot_current_yaw, robot_angle_to_buffer_point_yaw);
		cout << "dist_buffer_point_yaw now: " << dist_buffer_point_yaw << endl;
		// double angle_dist = angles::shortest_angular_distance(dist_buffer_point_yaw, -M_PI * 0.5);
		double angle_dist = dist_buffer_point_yaw;
		cout << "angle_dist: " << angle_dist << endl;
		pre_time = now_time;
		if(std::abs(angle_dist) < 0.05)
		{
			navigate_state_ = NavigateStates::MOVE_TO_BUFFER_POINT;
		}
		else
		{
			bound_rotation(angle_dist);
			if(std::abs(angle_dist) < 0.1) // 0.1 => 0.8 => raw_vel output 0
			{
				angle_dist = std::copysign(0.1, angle_dist);
			}
			servo_vel->angular.z = angle_dist;
			std::cout << "angular.z: " << angle_dist << endl;
		}
		break;
	}
	case NavigateStates::MOVE_TO_BUFFER_POINT:
	{
		std::cout << "------------- MOVE_TO_BUFFER_POINT -------------\n";
		servo_vel = geometry_msgs::msg::Twist();
		now_time = clock_->now().seconds();
		double dt = now_time - pre_time;
		// cout << "dist_buffer_point pre: " << dist_buffer_point << endl;
		// double delta_y = dt * raw_vel_msg.linear_x;
		dist_buffer_point += dt * raw_vel_msg.linear_x;
		pre_time = now_time;
		double dist_y = dist_buffer_point;
		cout << "raw_vel.linear_x: " << raw_vel_msg.linear_x << endl;
		cout << "dt: " << dt << endl;
		cout << "dist_buffer_point now: " << dist_buffer_point << endl;
		cout << "dist_y: " << dist_y << endl;
		if (std::abs(dist_y) < 0.05)
		{
			navigate_state_ = NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION;
		}
		else
		{
			double translate_velocity = dist_y;
			if (std::abs(translate_velocity) > max_translation_) {
				translate_velocity = max_translation_;
			}
			servo_vel->linear.x = -translate_velocity;
			cout << "linear.x: " << translate_velocity << endl;
		}
		break;
	}
	case NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION:
	{
		std::cout << "------------- ANGLE_TO_X_POSITIVE_ORIENTATION -------------\n";
		servo_vel = geometry_msgs::msg::Twist();
		now_time = clock_->now().seconds();
		double dt = now_time - pre_time;
		// dist_buffer_point_yaw += dist_buffer_point_yaw * dt;
		robot_current_yaw += raw_vel_msg.angular_z * dt;
		pre_time = now_time;
		double dist_yaw = angles::shortest_angular_distance(robot_current_yaw, 0);
		if(std::abs(dist_yaw) < 0.05)
		{
			navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
		}
		else
		{
			bound_rotation(dist_yaw);
			if(std::abs(dist_yaw) < 0.1) // 0.1 => 0.8 => raw_vel output 0
			{
				dist_yaw = std::copysign(0.1, dist_yaw);
			}
			servo_vel->angular.z = dist_yaw;
		}
		break;
	}
	case NavigateStates::ANGLE_TO_GOAL:
	{
		std::cout << "------------- ANGLE_TO_GOAL -------------\n";
		const GoalPoint & gp = goal_points_.front();

		std::cout << "goal  => "
		          << " x: " << gp.x << " y: " << gp.y
		          << " yaw: " << gp.theta << std::endl;
		std::cout << "robot => "
		        // << " x: " << current_pose.getOrigin().getX()
		        // << " y: " << current_pose.getOrigin().getY()
		          << " yaw: " << current_angle << std::endl;

		double delta_y, delta_x;
		delta_y = std::abs(gp.y - current_position.getY());
		delta_x = std::abs(gp.x - current_position.getX());
		double dist_to_goal = std::hypot(delta_x, delta_y);
		if (dist_to_goal <= gp.radius || delta_y < DIS_ERROR) {
			servo_vel = geometry_msgs::msg::Twist();
			navigate_state_ = NavigateStates::GO_TO_GOAL_POSITION;
		} else {
			double ang = diff_angle(gp, current_position, current_angle);
			double ang_save = ang;
			std::cout << "diff 1 angle: " << ang << std::endl;
			if (gp.drive_backwards) {
				// Angle is 180 from travel direction
				// ang = angles::normalize_angle(ang + M_PI);
				// std::cout << "norm angle: " << ang << std::endl;
			}
			bound_rotation(ang);
			std::cout << "bound angle: " << ang << std::endl;
			std::cout << "--------------------------------\n";
			servo_vel = geometry_msgs::msg::Twist();
			if (std::abs(ang_save) < TO_GOAL_ANGLE_CONVERGED) {
				navigate_state_ = NavigateStates::GO_TO_GOAL_POSITION;
				std::cout << " ******** change to state GO_TO_GOAL_POSITION ******** \n";
			} else {
				servo_vel->angular.z = ang;
			}
		}
		break;
	}
	case NavigateStates::GO_TO_GOAL_POSITION:
	{
		std::cout << "------------- GO_TO_GOAL_POSITION -------------\n";
		const GoalPoint & gp = goal_points_.front();
		std::cout << "goal  => "
		          << " x: " << gp.x << " y: " << gp.y
		          << " yaw: " << gp.theta << std::endl;
		std::cout << "robot => "
		        // << " x: " << current_pose.getOrigin().getX()
		        // << " y: " << current_pose.getOrigin().getY()
		          << " yaw: " << current_angle << std::endl;
		double delta_y, delta_x;
		delta_y = std::abs(gp.y - current_position.getY());
		delta_x = std::abs(gp.x - current_position.getX());
		double dist_to_goal = std::hypot(delta_x, delta_y);
		double ang = diff_angle(gp, current_position, current_angle);

		std::cout << "diff 2 angle: " << ang << std::endl;
		double abs_ang = std::abs(ang);
		if (gp.drive_backwards) {
			// Angle is 180 from travel direction
			// abs_ang = angles::normalize_angle(abs_ang + M_PI);
		}
		servo_vel = geometry_msgs::msg::Twist();
		// If robot is close enough to goal, move to final stage
		if (dist_to_goal < goal_points_.front().radius) {
			navigate_state_ = NavigateStates::GOAL_ANGLE;
			std::cout << "******** change to state GOAL_ANGLE ********  \n";
			// If robot angle has deviated too much from path, reset
		} else if (abs_ang > GO_TO_GOAL_ANGLE_TOO_FAR && delta_y > DIS_ERROR && (delta_x + delta_y) > DIS_ERROR2) {
			navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
			std::cout << "******** change to state ANGLE_TO_GOAL ********  \n";
			// If niether of above conditions met, drive towards goal
		} else {
			double translate_velocity = dist_to_goal;
			if (translate_velocity > max_translation_) {
				translate_velocity = max_translation_;
			}
			if (gp.drive_backwards) {
				translate_velocity *= -1;
			}
			if (std::abs(current_position.getX()) < NEAR_POSITION_X)
			{
				translate_velocity = -NEAR_LINEAR_X;
			}
			servo_vel->linear.x = translate_velocity;
			std::cout << "linear_x: " << translate_velocity << std::endl;
			double angle_dist = angles::shortest_angular_distance(current_angle, 0);
			if(std::abs(current_position.getX()) < NEAR_POSITION_X && std::abs(angle_dist) > NEAR_ANGULAR)
			{
				servo_vel->angular.z = angle_dist;
				std::cout << "angle_: " << angle_dist << std::endl;
			} else
			{
				if (abs_ang > GO_TO_GOAL_APPLY_ROTATION_ANGLE) {
					bound_rotation(ang);
					servo_vel->angular.z = ang;
					std::cout << "angle__: " << ang << std::endl;
				}
			}

		}
		break;
	}
	case NavigateStates::GOAL_ANGLE:
	{
		std::cout << "***********************************\n";
		std::cout << "***********************************\n";
		std::cout << "------------- GOAL_ANGLE -------------\n";
		const GoalPoint & gp = goal_points_.front();
		std::cout << "goal  => "
		          << " x: " << gp.x << " y: " << gp.y
		          << " yaw: " << gp.theta << std::endl;
		std::cout << "robot => "
		          << " x: " << current_pose.getOrigin().getX()
		          << " y: " << current_pose.getOrigin().getY()
		          << " yaw: " << current_angle << std::endl;
		double ang = angles::shortest_angular_distance(current_angle, gp.theta);
		bound_rotation(ang);
		std::cout << "diff 3 angle: " << ang << std::endl;
		if (std::abs(ang) > GOAL_ANGLE_CONVERGED) {
			servo_vel = geometry_msgs::msg::Twist();
			servo_vel->angular.z = ang;
		} else {
			goal_points_.pop_front();
			std::cout << "============ pop goal============" << std::endl;
			if (goal_points_.size() > 0) {
				servo_vel = geometry_msgs::msg::Twist();
				navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
				std::cout << "******** change to state ANGLE_TO_GOAL ********  \n";
			}
		}
		break;
	}
	}
	time_end = std::chrono::high_resolution_clock::now();
	time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
	// std::cout << "cost " << time_cost  << " ms." << std::endl;
	// if(time_cost < time_interval)
	// {
	//   usleep((time_interval- time_cost) * 1000);
	// }
	return servo_vel;
}

private:
enum class NavigateStates
{
	LOOKUP_ARUCO_MARKER,
	ANGLE_TO_BUFFER_POINT,
	MOVE_TO_BUFFER_POINT,
	ANGLE_TO_X_POSITIVE_ORIENTATION,
	ANGLE_TO_GOAL,
	GO_TO_GOAL_POSITION,
	GOAL_ANGLE
};

struct GoalPoint
{
	double x;
	double y;
	double theta;
	float radius;
	bool drive_backwards;
};

void bound_rotation(double & rotation_velocity)
{
	double abs_rot = std::abs(rotation_velocity);
	if (abs_rot > max_rotation_) {
		rotation_velocity = std::copysign(max_rotation_, rotation_velocity);
	} else if (abs_rot < MIN_ROTATION && abs_rot > 0.01) {
		// min speed if desire small non zero velocity
		rotation_velocity = std::copysign(MIN_ROTATION, rotation_velocity);
	}
}

double diff_angle(const GoalPoint & goal_pt, const tf2::Vector3 & cur_position, double cur_angle)
{

	double y = goal_pt.y - cur_position.getY();
	double x = goal_pt.x - cur_position.getX();
	double atan2_value = std::atan2(y, x);

	double result = angles::shortest_angular_distance(cur_angle, atan2_value);

	std::cout << "------caculate diff-------\n";

	std::cout << " gp.x: " << goal_pt.x << " gp.y: " << goal_pt.y << std::endl
	          << " cur.x: " << cur_position.getX() << " cur.y: " << cur_position.getY()<< std::endl;

	std::cout << "y       => " << y << std::endl;
	std::cout << "x       => " << x << std::endl;
	std::cout << "atan2   => " << atan2_value << std::endl;
	std::cout << "cur_ang => " << cur_angle << std::endl;
	std::cout << "dist    => " << result << std::endl;

	return result;
}

std::mutex mutex_;
std::deque<GoalPoint> goal_points_;
NavigateStates navigate_state_;
double max_rotation_;
double max_translation_;
const double MIN_ROTATION {0.01};
// const double TO_GOAL_ANGLE_CONVERGED {0.03};
const double TO_GOAL_ANGLE_CONVERGED {0.10};
const double GO_TO_GOAL_ANGLE_TOO_FAR {0.15};
// const double GO_TO_GOAL_APPLY_ROTATION_ANGLE {0.02};
const double GO_TO_GOAL_APPLY_ROTATION_ANGLE {0.10};
// const double GOAL_ANGLE_CONVERGED {0.02};
const double GOAL_ANGLE_CONVERGED {0.15};
const double LOOKUP_MARKER_CONVERGED {0.1};

std::chrono::high_resolution_clock::time_point time_start;
std::chrono::high_resolution_clock::time_point time_end;
int64_t time_cost;
// int64_t time_interval = 100;

double DIS_ERROR = 0.05;
double DIS_ERROR2 = 0.3;
double NEAR_POSITION_X = 0.32 + 0.10;
double NEAR_ANGULAR = 0.05;
double NEAR_LINEAR_X = 0.02;

double dist_buffer_point;
double dist_buffer_point_yaw;
double pre_time;
double now_time;
double first_sees_dock_time;
bool first_sees_dock = true;
double thre_angle_diff = 0.30; // 0.4461565280195475968735605160853 <= tan(32-arctan2(0.12/(0.32+0.1+0.5))

// buffer_goal_point
double buffer_goal_point_x = -(0.32 + 0.1 + 0.5 + 1); // docked,low_vel_dist,first_goal_dist, buffer_goal_dist
double buffer_goal_point_y = 0.0;
double buffer_goal_point_theta = 0;
double robot_angle_to_buffer_point_yaw;
double robot_current_yaw;
};

}  // namespace irobot_create_nodes
#endif   // IROBOT_CREATE_NODES__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_
