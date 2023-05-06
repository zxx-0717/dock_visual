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
    : pose(p), radius(r), drive_backwards(db) {}
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
    navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
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
    const tf2::Transform & current_pose)
  {
    time_start = std::chrono::high_resolution_clock::now();
    BehaviorsScheduler::optional_output_t servo_vel;
    const std::lock_guard<std::mutex> lock(mutex_);
    if (goal_points_.size() == 0) {
      return servo_vel;
    }
    double current_angle = tf2::getYaw(current_pose.getRotation());

    // correct angle to  right-orintation (pi/2,pi), left-orintation(-pi,-pi/2)
    // if (current_angle > -M_PI * 0.5 && current_angle < 0) // right
    // {
    //   current_angle = M_PI * 0.5 + std::abs(current_angle);  // (0,-pi/2) => (pi/2,pi)
    //   std::cout << "" << std::endl;
    // }
    // else if(current_angle < -M_PI * 0.5 && current_angle > -M_PI) // left
    // {
    //   current_angle = -M_PI * 1.5 - current_angle ; // (-pi/2,-pi) => (-pi, -pi/2)
    // }

    // correct angle to right-orintation (pi/2-pi), left-orintation (-pi, -pi/2)
    if (current_angle > M_PI * 0.5 && current_angle < M_PI) // left
    {
      current_angle = M_PI * 1.5 - current_angle;  // (pi, pi/2) => (pi/2, pi)
    }
    else if (current_angle > 0 && current_angle < M_PI * 0.5)
    {
      current_angle = -M_PI * 0.5 - current_angle; // (pi/2, 0) => (-pi, -pi/2)
    }


    const tf2::Vector3 & current_position = current_pose.getOrigin();
    // Generate velocity based on current position and next goal point looking for convergence
    // with goal point based on radius.
    switch (navigate_state_) {
      case NavigateStates::LOOKUP_ARUCO_MARKER:
      {
        std::cout << "------------- LOOKUP_ARUCO_MARKER -------------\n";
        
        break;
      }
      case NavigateStates::ANGLE_TO_GOAL:
        {
          std::cout << "------------- ANGLE_TO_GOAL -------------\n";
          const GoalPoint & gp = goal_points_.front();

          std::cout << "goal  => " 
          // << " x: " << gp.x << " y: " << gp.y 
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
          // << " x: " << gp.x << " y: " << gp.y 
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
            servo_vel->linear.x = translate_velocity;
            std::cout << "linear_x: " << translate_velocity << std::endl;
            if (abs_ang > GO_TO_GOAL_APPLY_ROTATION_ANGLE) {
              bound_rotation(ang);
              servo_vel->angular.z = ang;
              std::cout << "angle: " << ang << std::endl;
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
    std::cout << "cost " << time_cost  << " ms." << std::endl;
    if(time_cost < time_interval)
    {
      usleep((time_interval- time_cost) * 1000);
    }
    return servo_vel;
  }

private:
  enum class NavigateStates
  {
    ANGLE_TO_GOAL,
    GO_TO_GOAL_POSITION,
    GOAL_ANGLE,
    LOOKUP_ARUCO_MARKER
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
  const double MIN_ROTATION {0.02};
  // const double TO_GOAL_ANGLE_CONVERGED {0.03};
  const double TO_GOAL_ANGLE_CONVERGED {0.15};
  const double GO_TO_GOAL_ANGLE_TOO_FAR {0.20};
  // const double GO_TO_GOAL_APPLY_ROTATION_ANGLE {0.02};
  const double GO_TO_GOAL_APPLY_ROTATION_ANGLE {0.15};
  // const double GOAL_ANGLE_CONVERGED {0.02};
  const double GOAL_ANGLE_CONVERGED {0.05};

  std::chrono::high_resolution_clock::time_point time_start;
  std::chrono::high_resolution_clock::time_point time_end;
  int64_t time_cost;
  int64_t time_interval = 100;

  double DIS_ERROR = 0.05;
  double DIS_ERROR2 = 0.3;

};

}  // namespace irobot_create_nodes
#endif   // IROBOT_CREATE_NODES__MOTION_CONTROL__SIMPLE_GOAL_CONTROLLER_HPP_
