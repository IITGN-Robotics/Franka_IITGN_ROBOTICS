#include <cmath>
#include <memory>
#include <array>
#include <ros/ros.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>

namespace franka_iitgn {

class CartesianControllerBack : public controller_interface::MultiInterfaceController<
    franka_hw::FrankaPoseCartesianInterface,
    franka_hw::FrankaStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override
  {
    // Get the Cartesian pose interface.
    cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
    if (cartesian_pose_interface_ == nullptr)
    {
      ROS_ERROR("CartesianControllerBack: Could not get Cartesian Pose interface from hardware");
      return false;
    }
    
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("CartesianControllerBack: Could not get parameter arm_id");
      return false;
    }
    
    try
    {
      cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
          cartesian_pose_interface_->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("CartesianControllerBack: Exception getting Cartesian handle: " << e.what());
      return false;
    }
    
    // Get the state interface to access robot state.
    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR("CartesianControllerBack: Could not get state interface from hardware");
      return false;
    }
    
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("CartesianControllerBack: Exception getting state handle: " << e.what());
      return false;
    }
    
    return true;
  }
  
  void starting(const ros::Time& /*time*/) override
  {
    elapsed_time_ = ros::Duration(0.0);

    // Get the initial pose from the robot state.
    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    
    // initial_pose_ = {
    //   0.026864981627663487, 0.0018450239563343619, -0.9996373514531178, 0.0, 
    //   -0.9993944642639335, 0.02216958143148038, -0.026817535852803998, 0.0, 
    //   0.02211206342575467, 0.9997525221417619, 0.002439492209744848, 0.0, 
    //   0.6301460240960834, -0.011547089446797762, 0.10834631093634614, 1.0
    // };

    // Compute the target pose by moving 5 cm back along the y–axis.
    // (If you prefer to move along x, change index 13 to index 12.)
    // target_pose_ = initial_pose_;
    // target_pose_[13] -= 0.015;

    target_pose_={
      0.007641657694294035, 0.9995753551542758, 0.028119036387209476, 0.0,
       0.9994941097819746, -0.008503143029289562, 0.030646155406829714, 0.0,
       0.030872242919853132, 0.027870624766466567, -0.999134692067203, 0.0,
        0.5176954875167306, -0.21744742886616145, 0.28257896299540086, 1.0
    };

    // target_pose_ = {
    //   0.026864981627663487, 0.0018450239563343619, -0.9996373514531178, 0.0, 
    //   -0.9993944642639335, 0.02216958143148038, -0.026817535852803998, 0.0, 
    //   0.02211206342575467, 0.9997525221417619, 0.002439492209744848, 0.0, 
    //   0.6301460240960834, -0.031547089446797762, 0.10834631093634614, 1.0
    // };
    
    reached_target_ = false;
    ROS_INFO("CartesianControllerBack: Holding for 5 seconds before moving 5 cm back.");
  }
  
  void update(const ros::Time& /*time*/, const ros::Duration& period) override
  {
    elapsed_time_ += period;
    const double delay_time = 5.0;  // 5-second delay before starting the motion

    // During the delay period, hold the initial pose.
    if (elapsed_time_.toSec() < delay_time)
    {
      cartesian_pose_handle_->setCommand(initial_pose_);
      return;
    }
    
    // Compute effective time after the delay.
    double effective_time = elapsed_time_.toSec() - delay_time;
    const double total_time = 1.0; // Duration (seconds) for the motion
    std::array<double, 16> new_pose;
    
    // Smooth interpolation (cosine profile) between initial_pose_ and target_pose_
    double t = std::min(1.0, effective_time / total_time);
    t = 0.5 * (1.0 - std::cos(t * M_PI));
    
    for (size_t i = 0; i < 16; i++)
    {
      new_pose[i] = (1 - t) * initial_pose_[i] + t * target_pose_[i];
    }
    
    // Send the command.
    cartesian_pose_handle_->setCommand(new_pose);
    
    if (t >= 1.0 && !reached_target_)
    {
      reached_target_ = true;
      ROS_INFO("CartesianControllerBack: Reached target pose (5 cm back).");
    }
  }
  
  ~CartesianControllerBack() override {}

private:
  // Cartesian command interface and handle.
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  
  // State interface and handle.
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  
  // Storage for the initial and target poses.
  std::array<double, 16> initial_pose_;
  std::array<double, 16> target_pose_;
  
  bool reached_target_{false};
  ros::Duration elapsed_time_;
};

}  // namespace franka_iitgn

PLUGINLIB_EXPORT_CLASS(franka_iitgn::CartesianControllerBack, controller_interface::ControllerBase)

// #include <cmath>
// #include <memory>
// #include <array>
// #include <ros/ros.h>
// #include <controller_interface/controller_base.h>
// #include <controller_interface/multi_interface_controller.h>
// #include <franka_hw/franka_cartesian_command_interface.h>
// #include <franka_hw/franka_state_interface.h>
// #include <hardware_interface/robot_hw.h>
// #include <pluginlib/class_list_macros.h>

// namespace franka_iitgn {

// class CartesianControllerBack : public controller_interface::MultiInterfaceController<
//     franka_hw::FrankaPoseCartesianInterface,
//     franka_hw::FrankaStateInterface>
// {
// public:
//   bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override
//   {
//     // Get the Cartesian pose interface.
//     cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
//     if (cartesian_pose_interface_ == nullptr)
//     {
//       ROS_ERROR("CartesianControllerBack: Could not get Cartesian Pose interface from hardware");
//       return false;
//     }
    
//     std::string arm_id;
//     if (!node_handle.getParam("arm_id", arm_id))
//     {
//       ROS_ERROR("CartesianControllerBack: Could not get parameter arm_id");
//       return false;
//     }
    
//     try
//     {
//       cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
//           cartesian_pose_interface_->getHandle(arm_id + "_robot"));
//     }
//     catch (const hardware_interface::HardwareInterfaceException& e)
//     {
//       ROS_ERROR_STREAM("CartesianControllerBack: Exception getting Cartesian handle: " << e.what());
//       return false;
//     }
    
//     // Get the state interface to access robot state.
//     auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
//     if (state_interface == nullptr)
//     {
//       ROS_ERROR("CartesianControllerBack: Could not get state interface from hardware");
//       return false;
//     }
    
//     try
//     {
//       state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
//           state_interface->getHandle(arm_id + "_robot"));
//     }
//     catch (const hardware_interface::HardwareInterfaceException& e)
//     {
//       ROS_ERROR_STREAM("CartesianControllerBack: Exception getting state handle: " << e.what());
//       return false;
//     }
    
//     return true;
//   }
  
//   void starting(const ros::Time& /*time*/) override
//   {
//     elapsed_time_ = ros::Duration(0.0);
//     // Get the initial pose from the robot state.
//     initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    
//     // Compute the target pose by moving 5 cm back along the y–axis.
//     // (If you prefer to move along x, change index 13 to index 12.)
//     target_pose_ = initial_pose_;
//     target_pose_[13] -= 0.05;
    
//     reached_target_ = false;
//     ROS_INFO("CartesianControllerBack: Starting move 5 cm back.");
//   }
  
//   void update(const ros::Time& /*time*/, const ros::Duration& period) override
//   {
//     elapsed_time_ += period;
//     const double total_time = 3.0; // Total duration (seconds) of the motion
//     std::array<double, 16> new_pose;
    
//     // Compute a smooth interpolation (cosine profile) between the initial and target pose.
//     double t = std::min(1.0, elapsed_time_.toSec() / total_time);
//     t = 0.5 * (1.0 - std::cos(t * M_PI));
    
//     for (size_t i = 0; i < 16; i++)
//     {
//       new_pose[i] = (1 - t) * initial_pose_[i] + t * target_pose_[i];
//     }
    
//     // Send the command.
//     cartesian_pose_handle_->setCommand(new_pose);
    
//     if (t >= 1.0 && !reached_target_)
//     {
//       reached_target_ = true;
//       ROS_INFO("CartesianControllerBack: Reached target pose (5 cm back).");
//     }
//   }
  
//   ~CartesianControllerBack() override {}

// private:
//   // Cartesian command interface and handle.
//   franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
//   std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  
//   // State interface and handle.
//   std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  
//   // Storage for the initial and target poses.
//   std::array<double, 16> initial_pose_;
//   std::array<double, 16> target_pose_;
  
//   bool reached_target_{false};
//   ros::Duration elapsed_time_;
// };

// }  // namespace franka_iitgn

// PLUGINLIB_EXPORT_CLASS(franka_iitgn::CartesianControllerBack, controller_interface::ControllerBase)
