#include <cmath>
#include <memory>
#include <array>
#include <thread>
#include <fstream>
#include <ros/ros.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

namespace franka_iitgn
{

class CartesianController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaPoseCartesianInterface,
                                        franka_hw::FrankaStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  ~CartesianController() override;  // Destructor to close the file

private:
  // Cartesian command interface & handle.
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

  // State interface & handle for joint data.
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  // CSV file stream for logging joint data.
  std::ofstream csv_file_;

  // Timing and pose storage.
  ros::Duration elapsed_time_;
  std::array<double, 16> initial_pose_;

  // Phase tracking flags.
  bool gripper_command_sent_ = false;
  bool reached_A_ = false;

  // Helper function for gripper command.
  void sendGraspCommand();
};

bool CartesianController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  // Get the Cartesian pose interface.
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr)
  {
    ROS_ERROR("CartesianController: Could not get Cartesian Pose interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id))
  {
    ROS_ERROR("CartesianController: Could not get parameter arm_id");
    return false;
  }

  try
  {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("CartesianController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  // Get the state interface to access joint data.
  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr)
  {
    ROS_ERROR("CartesianController: Could not get state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("CartesianController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianController::starting(const ros::Time& /*time*/)
{
  elapsed_time_ = ros::Duration(0.0);
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  gripper_command_sent_ = false;
  reached_A_ = false;

  // Open CSV file for writing using an absolute path.
  csv_file_.open("/tmp/joint_data.csv", std::ios::out);
  if (!csv_file_.is_open())
  {
    ROS_ERROR("CartesianController: Could not open /tmp/joint_data.csv for writing");
  }
  else
  {
    ROS_INFO("CartesianController: Successfully opened /tmp/joint_data.csv for writing");
    // Write CSV header: time, joint positions (q0 ... q6), joint velocities (dq0 ... dq6), joint efforts (tau0 ... tau6)
    csv_file_ << "time";
    for (int i = 0; i < 7; ++i)
      csv_file_ << ",q" << i;
    for (int i = 0; i < 7; ++i)
      csv_file_ << ",dq" << i;
    for (int i = 0; i < 7; ++i)
      csv_file_ << ",tau" << i;
    csv_file_ << "\n";
    csv_file_.flush();  // Force the header to be written
  }
}

void CartesianController::update(const ros::Time& time, const ros::Duration& period)
{
  elapsed_time_ += period;

  // Log joint data to CSV.
  // Note: The Franka Panda has 7 joints.
  if (csv_file_.is_open())
  {
    auto robot_state = state_handle_->getRobotState();
    csv_file_ << time.toSec();
    for (int i = 0; i < 7; ++i)
      csv_file_ << "," << robot_state.q[i];
    for (int i = 0; i < 7; ++i)
      csv_file_ << "," << robot_state.dq[i];
    for (int i = 0; i < 7; ++i)
      csv_file_ << "," << robot_state.tau_J[i];
    csv_file_ << "\n";
    csv_file_.flush();  // Flush after each update to ensure data is written
  }

  // Define target transforms.
  std::array<double, 16> transformA = {
    0.037103723254654164, -0.004210240370928065, -0.9993025334542683, 0.0, 
    -0.9991600269633812, 0.01724863315255761, -0.03717110362044751, 0.0, 
    0.017393102684409384, 0.9998423668356913, -0.003566715493608652, 0.0, 
    0.6301851878236703, -0.061442542074913797, 0.10749134311310744, 1.0
  };

  std::array<double, 16> transformB = transformA;
  transformB[13] += 0.05; // Adjust the y-component

  const double total_time_A = 3.0;
  const double wait_time_A  = 5.0;
  const double total_time_B = 2.0;

  std::array<double, 16> new_pose;

  if (!reached_A_)
  {
    double t_A = std::min(1.0, elapsed_time_.toSec() / total_time_A);
    t_A = 0.5 * (1.0 - std::cos(t_A * M_PI));
    for (size_t i = 0; i < 16; i++)
    {
      new_pose[i] = (1 - t_A) * initial_pose_[i] + t_A * transformA[i];
    }
    if (elapsed_time_.toSec() >= total_time_A)
    {
      reached_A_ = true;
      elapsed_time_ = ros::Duration(0.0);
      ROS_INFO("Reached transformA. Entering hold phase.");
    }
  }
  else if (elapsed_time_.toSec() < wait_time_A)
  {
    new_pose = transformA;
  }
  else
  {
    double t_B = std::min(1.0, (elapsed_time_.toSec() - wait_time_A) / total_time_B);
    t_B = 0.5 * (1.0 - std::cos(t_B * M_PI));
    for (size_t i = 0; i < 16; i++)
    {
      new_pose[i] = (1 - t_B) * transformA[i] + t_B * transformB[i];
    }
  }
  cartesian_pose_handle_->setCommand(new_pose);
}

void CartesianController::sendGraspCommand()
{
  ROS_INFO("Sending gripper command...");
}

CartesianController::~CartesianController()
{
  // Close the CSV file when the controller is destroyed.
  if (csv_file_.is_open())
  {
    csv_file_.close();
  }
}

} // namespace franka_iitgn

// Register the controller as a plugin with pluginlib.
PLUGINLIB_EXPORT_CLASS(franka_iitgn::CartesianController, controller_interface::ControllerBase)
