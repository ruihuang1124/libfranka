//
// Created by ray on 5/17/22.
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
 * @example joint_impedance_control.cpp
 * An example showing a joint impedance type control that executes a Cartesian motion in the shape
 * of a circle. The example illustrates how to use the internal inverse kinematics to map a
 * Cartesian trajectory to joint space. The joint space target is tracked by an impedance control
 * that additionally compensates coriolis terms using the libfranka model library with a customized gravity direction.
 * This example also serves to compare commanded vs. measured torques. The results are printed from a separate
 * thread to avoid blocking print functions in the real-time loop.
 */

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
//  if (argc != 2) {
//    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
//    return -1;
//  }
  // Set and initialize trajectory parameters.
  const double radius = 0.05;
  const double vel_max = 0.25;
  const double acceleration_time = 2.0;
  const double run_time = 20.0;
  // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = 10.0;

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
          std::array<double, 7> tau_error{};
          double error_rms(0.0);
          std::array<double, 7> tau_d_actual{};
          for (size_t i = 0; i < 7; ++i) {
            tau_d_actual[i] = print_data.tau_d_last[i] + print_data.gravity[i];
            tau_error[i] = tau_d_actual[i] - print_data.robot_state.tau_J[i];
            error_rms += std::pow(tau_error[i], 2.0) / tau_error.size();
          }
          error_rms = std::sqrt(error_rms);

          // Print data to console
          std::cout << "tau_error [Nm]: " << tau_error << std::endl
                    << "tau_commanded [Nm]: " << tau_d_actual << std::endl
                    << "tau_measured [Nm]: " << print_data.robot_state.tau_J << std::endl
                    << "root mean square of tau_error [Nm]: " << error_rms << std::endl
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

    // First move the robot to a suitable joint configuration
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

    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
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

    // Set gains for the joint impedance control.
    // Stiffness
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
    // Customized_gravity_direction calculated from the customized installation way. left arm:
    const std::array<double, 3> customized_gravity_direction = {7.5439190864562988, 6.2554783821105957, -0.4408954679965973};

    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback =
            [&print_data, &model, k_gains, d_gains, customized_gravity_direction](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
      // Read current coriolis terms from model.
      std::array<double, 7> coriolis = model.coriolis(state);
      std::array<double, 7> normal_gravity = model.gravity(state);
      std::array<double, 7> customized_gravity = model.gravity(state,customized_gravity_direction);

      // Compute torque command from joint impedance control law.
      // Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
      // time step delay.
      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] =
            k_gains[i] * (state.q_d[i] - state.q[i]) - d_gains[i] * state.dq[i] + coriolis[i] + customized_gravity[i] - normal_gravity[i];
      }

      // The following line is only necessary for printing the rate limited torque. As we activated
      // rate limiting for the control loop (activated by default), the torque would anyway be
      // adjusted!
      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

      // Update data to print.
      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.robot_state = state;
        print_data.tau_d_last = tau_d_rate_limited;
        print_data.gravity = model.gravity(state,customized_gravity_direction);
        print_data.mutex.unlock();
      }

      // Send torque command.
      return tau_d_rate_limited;
    };

    // Start real-time control loop.
    robot.control(impedance_control_callback, cartesian_pose_callback);

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
