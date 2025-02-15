#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper> first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}


void CartesianPoseExampleController::update(const ros::Time& /* time */, const ros::Duration& period) {
  elapsed_time_ += period;

  // Get the current pose
  std::array<double, 16> current_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  // Define intermediate and final target poses
  std::array<double, 16> transformA = {
      0.10449221980874912, 0.019322751147294544, -0.9943379571649662, 0.0, 
      -0.9944936362100315, 0.010056875715249063, -0.10431314663097752, 0.0, 
      0.007984316553712512, 0.9997627171097679, 0.02026721901159892, 0.0, 
      0.6109069949392059, -0.07181951314839465, 0.37899845272559685, 1.0

      // 0.027784789235779928, 0.01728909277597348, -0.9994643858235098, 0.0, 
      // -0.9992946343556173, 0.025749102203999293, -0.027334653010775534, 0.0, 
      // 0.025262720132630645, 0.9995189197677782, 0.017992331637506798, 0.0, 
      // 0.5712296747257297, -0.1287586081991947, 0.10797833918146689, 1.0
  };

  std::array<double, 16> transformB = transformA;
  transformB[12]  += 0.0; // Move +5cm in X direction
  transformB[13]  += 0.1;  // No change in Y
  transformB[14] += 0.0;  // No change in Z

  // Motion timing
  double total_time_A = 50.0;  // Time to reach transformA
  double wait_time_A = 3.0;   // Wait at transformA
  double total_time_B = 3.0;  // Time to reach transformB

  static bool reached_A = false;  // Flag to check if we reached transformA
  static std::array<double, 16> transformA_actual;  // Store the actual reached pose

  std::array<double, 16> new_pose;

  if (!reached_A) {
    // Phase 1: Home → transformA
    double t_A = std::min(1.0, elapsed_time_.toSec() / total_time_A);
    t_A = 0.5 * (1.0 - std::cos(t_A * M_PI)); // Smooth acceleration

    ROS_INFO_STREAM("Phase 1: Moving to transformA | t_A: " << t_A << " | elapsed_time_: " << elapsed_time_.toSec());

    for (size_t i = 0; i < 16; i++) {
      new_pose[i] = (1 - t_A) * current_pose[i] + t_A * transformA[i];
    }

    if (elapsed_time_.toSec() >= total_time_A) {
      // Store the actual pose at transformA and reset time
      transformA_actual = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      reached_A = true; // Mark as reached
      elapsed_time_ = ros::Duration(0.0); // Reset time for next phase
      ROS_INFO_STREAM("Reached transformA. Holding position for " << wait_time_A << " seconds.");
    }
  } else if (elapsed_time_.toSec() < wait_time_A) {
    // Phase 2: Wait at transformA
    new_pose = transformA_actual; // Hold the position
    ROS_INFO_STREAM("Phase 2: Waiting at transformA | elapsed_time_: " << elapsed_time_.toSec());
  } else {
    // Phase 3: transformA → transformB
    double t_B = std::min(1.0, (elapsed_time_.toSec() - wait_time_A) / total_time_B);
    t_B = 0.5 * (1.0 - std::cos(t_B * M_PI));

    ROS_INFO_STREAM("Phase 3: Moving to transformB | t_B: " << t_B << " | elapsed_time_: " << elapsed_time_.toSec());

    for (size_t i = 0; i < 16; i++) {
      new_pose[i] = (1 - t_B) * transformA_actual[i] + t_B * transformB[i];
    }
  }

  // Debugging output for new pose (printing only translation part)
  ROS_INFO_STREAM("New Pose Translation: x=" << new_pose[12] << ", y=" << new_pose[13] << ", z=" << new_pose[14]);

  // Send new pose to robot
  cartesian_pose_handle_->setCommand(new_pose);
}


}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerBase)