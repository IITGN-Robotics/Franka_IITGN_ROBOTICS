#include <ros/ros.h>
#include <franka_msgs/SetFullCollisionBehavior.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "set_collision_behavior");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<franka_msgs::SetFullCollisionBehavior>(
      "/franka_control/set_full_collision_behavior");

  franka_msgs::SetFullCollisionBehavior srv;

  // Set higher force/torque thresholds to allow pushing
  srv.request.lower_torque_thresholds_nominal = {100, 100, 100, 100, 100, 100, 100};
  srv.request.upper_torque_thresholds_nominal = {100, 100, 100, 100, 100, 100, 100};

  srv.request.lower_force_thresholds_nominal = {100, 100, 100, 100, 100, 100};
  srv.request.upper_force_thresholds_nominal = {100, 100, 100, 100, 100, 100};

  if (client.call(srv)) {
    ROS_INFO("Collision behavior updated successfully!");
  } else {
    ROS_ERROR("Failed to update collision behavior.");
    return 1;
  }

  return 0;
}
