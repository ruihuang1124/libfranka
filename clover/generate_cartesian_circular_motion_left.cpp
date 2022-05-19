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
#include <eigen3/Eigen/Dense>
#include <cmath>

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
  Eigen::Matrix<double, 4, 4> world_to_customized_transform_;
  world_to_customized_transform_ << 0.626009, -0.73698, 0.254887, 0, -0.12941, 0.224144, 0.965926, 0, -0.769003, -0.637663, 0.0449435, 0, 0, 0, 0, 1;
  Eigen::Affine3d world_to_install_rotation_transform_A(
      Eigen::Matrix4d::Map(world_to_customized_transform_.data()));
  /*
0.626009 -0.736984  0.254887         0
-0.12941  0.224144  0.965926         0
-0.769003 -0.637663 0.0449435        0
0         0         0         1
   */
  //  // Check whether the required arguments were passed.
  //  if (argc != 2) {
  //    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //    return -1;
  //  }
  // Set and initialize trajectory parameters.
  const double radius = 0.035;
  const double vel_max = 0.27;
  const double acceleration_time = 5.0;
  const double run_time = 30.0;
    // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = 1000.0;

  double vel_current = 0.0;
  double angle = 0.0;
  double time = 0.0;

    // Initialize data fields for the print thread.
  struct {
    std::mutex mutex;
    bool has_data;
    std::array<double, 7> tau_d_last;
    franka::RobotState robot_state;
    std::array<double, 7> gravity;
  } print_data{};
  std::atomic_bool running{true};

  // Start print thread.
  std::thread print_thread([print_rate, &print_data, &running]() {
    while (running) {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock()) {
        if (print_data.has_data) {
          // Print data to console
          std::cout << "tau_measured [Nm]: " << print_data.robot_state.tau_J << std::endl
                    << "Derivative of measured link-side joint torque sensor signals"<< print_data.robot_state.dtau_J<<std::endl
                    << "Desired Joint Velocity"<<print_data.robot_state.dq_d<<std::endl
                    << "-----------------------" << std::endl;
          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }
    }
  });

  try {
    // Connect to robot.
    franka::Robot robot("192.168.0.120");
    setDefaultBehavior(robot);

    // First move the robot to a suitable start joint configuration
  //  std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_start = {{-0.3, 0.2, 0, -2 * M_PI_4, 0, M_PI_2, 0.6}};
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

    std::array<double, 16> initial_pose_arm_frame;
    std::array<double, 16> initial_pose_world_frame;
    Eigen::Vector3d ee_position_world_base;
    Eigen::Vector3d ee_position_franka_base;

    // Define callback function to send Desired Cartesian pose goals.
    auto cartesian_pose_callback = [=, &time, &vel_current, &running, &angle, &initial_pose_arm_frame, &ee_position_world_base, &initial_pose_world_frame, &ee_position_franka_base, &print_data](
                                       const franka::RobotState& robot_state,
                                       franka::Duration period) -> franka::CartesianPose {
      // Update time.
      time += period.toSec();

      if (time == 0.0) {
        Eigen::Affine3d franka_to_ee_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_c.data()));
        Eigen::Affine3d world_to_ee_transform =
            world_to_install_rotation_transform_A * franka_to_ee_transform;
        ee_position_world_base = world_to_ee_transform.translation();
        // Read the initial pose to start the motion from in the first time step.
        initial_pose_arm_frame = robot_state.O_T_EE_c;
        initial_pose_world_frame = robot_state.O_T_EE_c;
        initial_pose_world_frame[12] = ee_position_world_base(0);
        initial_pose_world_frame[13] = ee_position_world_base(1);
        initial_pose_world_frame[14] = ee_position_world_base(2);
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

      double delta_x_world = radius * (1-std::cos(angle));
      double delta_y_world = radius * std::sin(angle);
      std::array<double, 16> start_pose_world_frame;
      start_pose_world_frame = initial_pose_world_frame;
      start_pose_world_frame[12] +=delta_x_world;
      start_pose_world_frame[13] +=delta_y_world;

      Eigen::Affine3d world_to_ee_transform_current(
          Eigen::Matrix4d::Map(start_pose_world_frame.data()));

      Eigen::Affine3d franka_to_ee_transform_current = world_to_install_rotation_transform_A.inverse() * world_to_ee_transform_current;

      ee_position_franka_base = franka_to_ee_transform_current.translation();

//      franka::CartesianPose pose_desired = initial_pose_arm_frame;
//      pose_desired.O_T_EE[13] += delta_y;
//      pose_desired.O_T_EE[14] += delta_z;

      franka::CartesianPose pose_desired = initial_pose_arm_frame;
      pose_desired.O_T_EE[12] = ee_position_franka_base(0);
      pose_desired.O_T_EE[13] = ee_position_franka_base(1);
      pose_desired.O_T_EE[14] = ee_position_franka_base(2);

            // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = robot_state;
        print_data.mutex.unlock();
      }


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
    if (print_thread.joinable()) {
    print_thread.join();
  }
  return 0;
}
