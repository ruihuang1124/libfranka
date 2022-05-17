//
// Created by ray on 5/4/22.
//
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "clover_common.h"

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

/**
 * @example generate_cartesian_circular_motion.cpp
 * An example executes a Cartesian motion in the shape
 * of a circle.
 */

int main(int argc, char** argv) {
  //  // Check whether the required arguments were passed.
  //  if (argc != 2) {
  //    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //    return -1;
  //  }
  // Set and initialize trajectory parameters.
  const double radius = 0.05;
  const double vel_max = 0.25;
  const double acceleration_time = 2.0;
  const double run_time = 20.0;

  double vel_current = 0.0;
  double angle = 0.0;
  double time = 0.0;

  std::atomic_bool running{true};

  try {
    // Connect to robot.
    franka::Robot robot("192.168.0.120");
    setDefaultBehavior(robot);

    // First move the robot to a suitable start joint configuration
    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    JointMotionGenerator move_to_start_generator(0.1, q_start);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(move_to_start_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0}},
        {{400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0}},
        {{400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0}},
        {{400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 400.0}},
        {{250.0, 250.0, 250.0, 250.0, 250.0, 250.0}}, {{250.0, 250.0, 250.0, 250.0, 250.0, 250.0}},
        {{250.0, 250.0, 250.0, 250.0, 250.0, 250.0}}, {{250.0, 250.0, 250.0, 250.0, 250.0, 250.0}});

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();

    std::array<double, 16> initial_pose;

    // Define callback function to send Desired Cartesian pose goals.
    auto cartesian_pose_callback = [=, &time, &vel_current, &running, &angle, &initial_pose](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {
      // Update time.
      time += period.toSec();

      if (time == 0.0) {
        // Read the initial pose to start the motion from in the first time step.
        initial_pose = robot_state.O_T_EE_c;
      }

      // Compute Cartesian velocity.
      if (vel_current < vel_max && time < run_time) {
        vel_current += period.toSec() * std::fabs(vel_max / acceleration_time);
      }
      if (vel_current > 0.0 && time > run_time) {
        vel_current -= period.toSec() * std::fabs(vel_max / acceleration_time);
      }
      vel_current = std::fmax(vel_current, 0.0);
      vel_current = std::fmin(vel_current, vel_max);

      // Compute new angle for our circular trajectory.
      angle += period.toSec() * vel_current / std::fabs(radius);
      if (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
      }

      // Compute relative y and z positions of desired pose.
      double delta_y = radius * (1 - std::cos(angle));
      double delta_z = radius * std::sin(angle);
      franka::CartesianPose pose_desired = initial_pose;
      pose_desired.O_T_EE[13] += delta_y;
      pose_desired.O_T_EE[14] += delta_z;

      // Send desired pose.
      if (time >= run_time + acceleration_time) {
        running = false;
        return franka::MotionFinished(pose_desired);
      }

      return pose_desired;
    };
    // Start real-time control loop to execute the cartesian circular shape motion.
    std::cout << "Robot will execute the cartesian circular shape motion from current pose."
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(cartesian_pose_callback);

    // Finally, move the robot to a suitable joint configuration
    std::array<double, 7> q_final = {{0.613, 0.696, -0.03416, -0.579, 0.0627, 2.129, 0.187}};
    JointMotionGenerator move_to_final_generator(0.1, q_final);
    std::cout << "Robot will move to final joint configuration." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(move_to_final_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
