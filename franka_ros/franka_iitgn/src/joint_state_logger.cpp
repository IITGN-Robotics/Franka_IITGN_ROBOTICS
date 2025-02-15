// DO NOT TOUCH THE CODE BELOW
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <string>
#include <vector>

class JointStateLogger {
public:
  JointStateLogger(ros::NodeHandle& nh)
    : nh_(nh), header_written_(false)
  {
    // Get CSV file path from parameter (default: /tmp/joint_data.csv)
    std::string file_path;
    nh_.param<std::string>("csv_file_path", file_path, "/home/iitgn-robotics/franka_ros_ws/src/franka_ros/franka_iitgn/data/joint_data.csv");

    csv_file_.open(file_path, std::ios::out);
    if (!csv_file_.is_open()) {
      ROS_ERROR("Failed to open CSV file: %s", file_path.c_str());
    } else {
      ROS_INFO("Opened CSV file: %s", file_path.c_str());
    }

    // Subscribe to the joint state topic.
    joint_state_sub_ = nh_.subscribe("/franka_state_controller/joint_states", 1000, 
                                       &JointStateLogger::jointStateCallback, this);
  }

  ~JointStateLogger(){
    if(csv_file_.is_open()){
      csv_file_.close();
    }
  }

private:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!csv_file_.is_open()) {
      ROS_ERROR_THROTTLE(5, "CSV file is not open; cannot log joint data.");
      return;
    }

    // Write header on the first callback.
    if (!header_written_) {
      // Header: time, then for each joint: name_pos, then name_vel, then name_eff.
      csv_file_ << "time";
      for (const auto& name : msg->name)
        csv_file_ << "," << name << "_pos";
      for (const auto& name : msg->name)
        csv_file_ << "," << name << "_vel";
      for (const auto& name : msg->name)
        csv_file_ << "," << name << "_eff";
      csv_file_ << "\n";
      header_written_ = true;
    }

    // Use the message header's timestamp if available; otherwise, use ros::Time::now().
    double t = (msg->header.stamp.isZero()) ? ros::Time::now().toSec() : msg->header.stamp.toSec();
    csv_file_ << t;

    // Write joint positions.
    for (const auto& pos : msg->position)
      csv_file_ << "," << pos;

    // Write joint velocities.
    for (const auto& vel : msg->velocity)
      csv_file_ << "," << vel;

    // Write joint efforts.
    for (const auto& eff : msg->effort)
      csv_file_ << "," << eff;

    csv_file_ << "\n";
    csv_file_.flush();  // Ensure data is written to disk immediately.
  }

  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  std::ofstream csv_file_;
  bool header_written_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_state_logger");
  ros::NodeHandle nh("~"); // private NodeHandle to allow parameters like ~csv_file_path
  JointStateLogger logger(nh);
  ros::spin();
  return 0;
}
