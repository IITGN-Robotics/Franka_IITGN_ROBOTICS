#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <thread>     

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>  

#include <control_msgs/GripperCommandAction.h>

namespace franka_example_controllers
{

class CartesianPosePointController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaPoseCartesianInterface,
                                        franka_hw::FrankaStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time&, const ros::Duration& period) override;

private:
  // Cartesian command interface & handle.
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

  // For interpolation timing.
  ros::Duration elapsed_time_;
  std::array<double, 16> initial_pose_;  // Saved in starting()

  // Phase–tracking flags.
  bool gripper_command_sent_ = false;
  bool reached_A_ = false;  // becomes true when interpolation to transformA completes

  // Gripper helper function using GraspAction to apply a force of 10N.
  void sendGraspCommand();
};

bool CartesianPosePointController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle)
{
  // Get Cartesian pose interface.
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr)
  {
    ROS_ERROR("CartesianPosePointController: Could not get Cartesian Pose interface from hardware");
    return false;
  }

  // Get arm_id parameter.
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id))
  {
    ROS_ERROR("CartesianPosePointController: Could not get parameter arm_id");
    return false;
  }

  try
  {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("CartesianPosePointController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  // Also check the state interface.
  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr)
  {
    ROS_ERROR("CartesianPosePointController: Could not get state interface from hardware");
    return false;
  }
  return true;
}

void CartesianPosePointController::starting(const ros::Time& /*time*/)
{
  // Save the starting pose so that interpolation remains fixed.
  elapsed_time_ = ros::Duration(0.0);
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  gripper_command_sent_ = false;
  reached_A_ = false;
}

void CartesianPosePointController::update(const ros::Time&, const ros::Duration& period)
{
  elapsed_time_ += period;

  // Define target transforms.
  std::array<double, 16> transformA = {
    0.004630059928271646, -0.012357494481277511, -0.9999129065306958, 0.0, 
    0.9997021957355872, -0.023900529292637845, 0.004924460623874138, 0.0, 
    -0.023959302527698317, -0.9996379629670421, 0.012243153902846526, 0.0, 
    0.5772780476593037, 0.115674569538702457, 0.11161566778977149, 1.0
  };

  std::array<double, 16> transformB = transformA;
  transformB[13] -= 0.1;  // Offset in Y

  // Timing parameters (adjust as desired).
  const double total_time_A = 3.0;  // Duration to reach transformA
  const double wait_time_A  = 5.;  // Hold time at transformA
  const double total_time_B = 2.0;  // Duration to move to transformB

  std::array<double, 16> new_pose;

  if (!reached_A_)
  {
    // Phase 1: Interpolate from the fixed starting pose to transformA.
    double t_A = std::min(1.0, elapsed_time_.toSec() / total_time_A);
    t_A = 0.5 * (1.0 - std::cos(t_A * M_PI));
    for (size_t i = 0; i < 16; i++) {
      new_pose[i] = (1 - t_A) * initial_pose_[i] + t_A * transformA[i];
    }
    if (elapsed_time_.toSec() >= total_time_A)
    {
      reached_A_ = true;
      elapsed_time_ = ros::Duration(0.0);  // Reset timer for Phase 2.
      ROS_INFO("Reached transformA. Entering hold phase.");
    }
  }
  else if (elapsed_time_.toSec() < wait_time_A)
  {
    // Phase 2: Hold at transformA.
    new_pose = transformA;
    // Launch the grasp command thread only once.
    if (!gripper_command_sent_)
    {
      gripper_command_sent_ = true;
      std::thread(&CartesianPosePointController::sendGraspCommand, this).detach();
    }
  }
  else
  {
    // Phase 3: Interpolate from transformA to transformB.
    double t_B = std::min(1.0, (elapsed_time_.toSec() - wait_time_A) / total_time_B);
    t_B = 0.5 * (1.0 - std::cos(t_B * M_PI));
    for (size_t i = 0; i < 16; i++) {
      new_pose[i] = (1 - t_B) * transformA[i] + t_B * transformB[i];
    }
  }

  cartesian_pose_handle_->setCommand(new_pose);
}

void CartesianPosePointController::sendGraspCommand()
{
  ROS_INFO("Sending GripperCommand to target 35mm width with specified force...");

  // Create an action client for the standard gripper command action.
  // (The action server name may differ depending on your setup.)
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client("/franka_gripper/gripper_action", true);

  // Wait for the action server to come up.
  if (!gripper_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_ERROR("GripperCommand action server not available");
    return;
  }

  // Create and fill in the goal.
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.01; // Target width in meters (35 mm)
  goal.command.max_effort = 100;  // Specify the force (e.g., 100 N). Must be > 0 to apply force.

  // Send the goal.
  gripper_client.sendGoal(goal);

  // Wait for the result (adjust the timeout as necessary).
  if (gripper_client.waitForResult(ros::Duration(3.0)))
  {
    ROS_INFO("GripperCommand executed successfully.");
  }
  else
  {
    ROS_WARN("GripperCommand timed out.");
  }
}

// void CartesianPosePointController::sendGraspCommand()
// {
//   ROS_INFO("Sending GraspAction command to apply 10 N force...");
//   actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_client("/franka_gripper/grasp", true);
//   if (!gripper_client.waitForServer(ros::Duration(5.0)))
//   {
//     ROS_ERROR("Gripper grasp action server not available");
//     return;
//   }
//   franka_gripper::GraspGoal grasp_goal;
//   grasp_goal.width = 0.001;           // Target gripper width (closed)
//   grasp_goal.epsilon.inner = 0.005;    // Tolerance parameters for a successful grasp
//   grasp_goal.epsilon.outer = 0.005;
//   grasp_goal.speed = 0.1;              // Speed of closing
//   grasp_goal.force = 100;            // Command a force of 10 N
  
//   gripper_client.sendGoal(grasp_goal);
//   if (gripper_client.waitForResult(ros::Duration(3.0)))
//   {
//     ROS_INFO("GraspAction command completed successfully.");
//   }
//   else
//   {
//     ROS_WARN("GraspAction command timed out.");
//   }
// }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPosePointController,
                       controller_interface::ControllerBase)