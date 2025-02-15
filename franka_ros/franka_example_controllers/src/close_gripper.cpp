#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>

void closeGripper() 
{ 
  ROS_INFO_STREAM("Closing the Gripper...");

  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_client("/franka_gripper/grasp", true);
  gripper_client.waitForServer();

  franka_gripper::GraspGoal grasp_goal;
  grasp_goal.width = 0.001;
  grasp_goal.epsilon.inner = 0.005;
  grasp_goal.epsilon.outer = 0.005;
  grasp_goal.speed = 0.1;
  grasp_goal.force = 100;
  
  gripper_client.sendGoal(grasp_goal);
  gripper_client.waitForResult(ros::Duration(3.0));

  if (gripper_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("Gripper successfully closed!");
  else
    ROS_WARN_STREAM("Failed to close the gripper.");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "close_gripper_node");
  ros::NodeHandle nh;
 
  closeGripper();

  return 0;
}

// #include <cmath>
// #include <memory>
// #include <stdexcept>
// #include <string>
// #include <array>

// #include <controller_interface/controller_base.h>
// #include <franka_hw/franka_cartesian_command_interface.h>
// #include <hardware_interface/hardware_interface.h>
// #include <pluginlib/class_list_macros.h>
// #include <ros/ros.h>

// #include <controller_interface/multi_interface_controller.h>
// #include <franka_hw/franka_state_interface.h>
// #include <hardware_interface/robot_hw.h>
// #include <ros/node_handle.h>
// #include <ros/time.h>

// #include <actionlib/client/simple_action_client.h>
// #include <franka_gripper/GraspAction.h>

// namespace franka_example_controllers
// {

// class CloseGripper : public controller_interface::MultiInterfaceController<
//                                         franka_hw::FrankaPoseCartesianInterface,
//                                         franka_hw::FrankaStateInterface> 
// {
// public:
//     bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
//     void starting(const ros::Time&) override;
//     void update(const ros::Time&, const ros::Duration& period) override;

// private:
//     franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
//     std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
//     ros::Duration elapsed_time_;

//     void closeGripper();  
// };

// bool CloseGripper::init(hardware_interface::RobotHW* robot_hardware, 
//                                        ros::NodeHandle& node_handle)
// {
//   // Get Cartesian Pose Interface
//   cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
//   if (cartesian_pose_interface_ == nullptr) 
//   {
//     ROS_ERROR("CloseGripper: Could not get Cartesian Pose interface from hardware");
//     return false;
//   }

//   // Get arm ID from ROS parameter 
//   std::string arm_id;
//   if (!node_handle.getParam("arm_id", arm_id)) 
//   {
//     ROS_ERROR("CloseGripper: Could not get parameter arm_id");
//     return false;
//   }

//   // Get Cartesian Pose Handle
//   try 
//   {
//     cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(cartesian_pose_interface_->getHandle(arm_id + "_robot"));
//   } 
//   catch (const hardware_interface::HardwareInterfaceException& e) 
//   {
//     ROS_ERROR_STREAM("CloseGripper: Exception getting Cartesian handle: " << e.what());
//     return false;
//   }

//   // Get Robot State Interface
//   auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
//   if (state_interface == nullptr) 
//   {
//     ROS_ERROR("CloseGripper: Could not get state interface from hardware");
//     return false;
//   } 

//   return true;
// }

// void CloseGripper::starting(const ros::Time&) 
// {
//   elapsed_time_ = ros::Duration(0.0);
// }

// void CloseGripper::update(const ros::Time&, const ros::Duration& period) {
//   elapsed_time_ += period;
//   closeGripper(); 
// }

// void CloseGripper::closeGripper() 
// { 
//   ROS_INFO_STREAM("Gripper Close");

//   actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_client("/franka_gripper/grasp", true);
//   gripper_client.waitForServer();

//   franka_gripper::GraspGoal grasp_goal;
//   grasp_goal.width = 0.02;
//   grasp_goal.epsilon.inner = 0.005;
//   grasp_goal.epsilon.outer = 0.005;
//   grasp_goal.speed = 0.1;
//   grasp_goal.force = 10;
  
//   gripper_client.sendGoal(grasp_goal);
//   gripper_client.waitForResult(ros::Duration(3.0));
// }

// }  // namespace franka_example_controllers

// PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CloseGripper,
//                        controller_interface::ControllerBase)