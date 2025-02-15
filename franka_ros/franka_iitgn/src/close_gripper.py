#!/usr/bin/env python
"""
A ROS node that uses the GraspAction to command the Franka gripper to grasp an object.
This action will close the gripper at a specified speed with a specified force until
the distance between the gripper fingers is within the acceptable tolerance (epsilon).

Parameters (set via ROS params or overridden at runtime):
  ~width:         Desired total gripper opening (in meters) at which to hold the object.
  ~epsilon_inner: Acceptable inner tolerance (in meters) for a successful grasp.
  ~epsilon_outer: Acceptable outer tolerance (in meters) for a successful grasp.
  ~speed:         The speed (in m/s) at which the gripper should close.
  ~force:         The force (in N) to be applied during the grasp.
  ~action_server: The name of the GraspAction server (default: "franka_gripper/grasp").

Usage:
  1. Launch the gripper node:
       roslaunch franka_gripper franka_gripper.launch robot_ip:=<your-ip>
  2. Run this node:
       rosrun <your_package> grasp_object.py
  3. Optionally override parameters, for example:
       rosrun <your_package> grasp_object.py _width:=0.03 _epsilon_inner:=0.005 _epsilon_outer:=0.005 _speed:=0.1 _force:=40.0
"""

import rospy
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal

def main():
    rospy.init_node('grasp_object_node', anonymous=False)

    # Retrieve parameters (with defaults)
    width         = rospy.get_param('~width', 0.025)         # desired total gripper opening [m]
    epsilon_inner = rospy.get_param('~epsilon_inner', 0.005)  # inner tolerance [m]
    epsilon_outer = rospy.get_param('~epsilon_outer', 0.005)  # outer tolerance [m]
    speed         = rospy.get_param('~speed', 0.1)           # closing speed [m/s]
    force         = rospy.get_param('~force', 1000.0)          # grasp force [N]
    action_server = rospy.get_param('~action_server', 'franka_gripper/grasp')
# 0.32 #h 0.25 m0.013
    # Create the action client for GraspAction.
    rospy.loginfo("Waiting for grasp action server [%s]...", action_server)
    client = actionlib.SimpleActionClient(action_server, GraspAction)
    client.wait_for_server()
    rospy.loginfo("Connected to grasp action server.")

    # Build the GraspGoal.
    goal = GraspGoal()
    # Set the fields directly, without a nested 'command' field.
    goal.width = width
    goal.epsilon.inner = epsilon_inner
    goal.epsilon.outer = epsilon_outer
    goal.speed = speed
    goal.force = force

    rospy.loginfo("Sending GraspGoal: width=%.3f, epsilon_inner=%.3f, epsilon_outer=%.3f, speed=%.3f, force=%.3f",
                  width, epsilon_inner, epsilon_outer, speed, force)
    client.send_goal(goal)

    # Wait for the grasp action to finish.
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Grasp action finished with result: %s", result)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



# #!/usr/bin/env python
# """
# A ROS node that continuously commands the Franka gripper to hold an object
# by applying a specified force without relying on the tight epsilon tolerance
# required by the GraspAction.

# This node uses the MoveAction interface (via control_msgs/GripperCommandActionGoal)
# to continuously publish a command. In this setup:
#   - The parameter 'target_width' is the total desired opening (object width).
#   - The node computes the per-finger command as target_width/2.
#   - 'max_effort' is the force (in Newtons) to apply.
  
# By continuously publishing the same command (at 10 Hz), the gripper is kept
# in the desired state, holding the object without needing 0.005 m precision.
  
# Usage:
#   1. Launch the gripper node:
#        roslaunch franka_gripper franka_gripper.launch robot_ip:=<your-ip>
#   2. Run this node (ensure it is executable):
#        rosrun <your_package_name> continuous_gripper_hold_no_epsilon.py
#   3. Optionally, override parameters:
#        rosrun <your_package_name> continuous_gripper_hold_no_epsilon.py _target_width:=0.04 _max_effort:=40.0
# """

# import rospy
# from control_msgs.msg import GripperCommandActionGoal

# def main():
#     rospy.init_node('continuous_gripper_hold_no_epsilon', anonymous=False)

#     # Publisher to the gripper action server goal topic.
#     pub = rospy.Publisher('/franka_gripper/gripper_action/goal',
#                           GripperCommandActionGoal,
#                           queue_size=1)
#     rate = rospy.Rate(10)  # Publish at 10 Hz

#     # Retrieve parameters:
#     # 'target_width' is the total opening width you wish to command (object width)
#     # 'max_effort' is the force to be applied.
#     target_width = rospy.get_param('~target_width', 0.035)  # default: 4 cm total width
#     max_effort   = rospy.get_param('~max_effort', 40.0)      # default force: 40 N

#     # Compute per-finger command: each finger's position is half of the total width.
#     finger_position = target_width / 2.0

#     rospy.loginfo("Holding object with total target_width=%.3f m "
#                   "(each finger commanded to %.3f m) and max_effort=%.3f N",
#                   target_width, finger_position, max_effort)

#     while not rospy.is_shutdown():
#         msg = GripperCommandActionGoal()
#         msg.header.stamp = rospy.Time.now()

#         # Set the command: note that the gripper expects each finger's position.
#         msg.goal.command.position = finger_position  
#         msg.goal.command.max_effort = max_effort

#         pub.publish(msg)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass


# #!/usr/bin/env python
# """
# A ROS node that continuously commands the Franka gripper to hold an object
# by applying a specified force.

# IMPORTANT:
# - Ensure the gripper node is already running:
#       roslaunch franka_gripper franka_gripper.launch robot_ip:=<your-ip>
# - Set the parameter 'target_width' to the TOTAL object's width.
#   NOTE: Since the gripper expects each finger's position (half the total width),
#   the command will be target_width/2.
# - 'max_effort' specifies the force (in Newtons) to apply.

# This node publishes at 10 Hz to the action topic so that the command is maintained.
# """

# import rospy
# from control_msgs.msg import GripperCommandActionGoal

# def main():
#     rospy.init_node('continuous_gripper_hold', anonymous=False)

#     # Publisher to the gripper action server goal topic.
#     pub = rospy.Publisher('/franka_gripper/gripper_action/goal',
#                           GripperCommandActionGoal,
#                           queue_size=1)
#     rate = rospy.Rate(10)  # Publish at 10 Hz

#     # Retrieve parameters:
#     # Set target_width to the total object's width (in meters).
#     target_width = rospy.get_param('~target_width', 0.035)  # default: 4 cm total width
#     max_effort   = rospy.get_param('~max_effort', 40.0)      # default force: 40 N

#     # Compute the per-finger command (each finger gets half the total width)
#     finger_position = target_width / 2.0

#     rospy.loginfo("Continuously holding object with total target_width=%.3f m "
#                   "(finger command=%.3f m) and max_effort=%.3f N",
#                   target_width, finger_position, max_effort)

#     while not rospy.is_shutdown():
#         # Build the command message.
#         msg = GripperCommandActionGoal()
#         msg.header.stamp = rospy.Time.now()

#         # Set the goal's command:
#         # Note: The gripper expects each finger's position (i.e., half of the total width).
#         msg.goal.command.position = finger_position
#         msg.goal.command.max_effort = max_effort

#         # Publish the command.
#         pub.publish(msg)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass



# #!/usr/bin/env python
# """
# A ROS node that commands the Franka gripper to hold an object by continuously applying force.

# This node uses the control_msgs/GripperCommandAction interface to send a grasp goal.
# When max_effort is set to a positive value, the gripper will try to grasp (i.e. hold)
# an object at the specified width. In this example, we command the gripper to a closed
# position (target_width=0.0) with a specified force.

# Before running this node, launch the gripper node (for example):
#     roslaunch franka_gripper franka_gripper.launch robot_ip:=<your-ip>
# """

# import rospy
# import actionlib
# from control_msgs.msg import GripperCommandAction, GripperCommandGoal

# def main():
#     rospy.init_node('franka_gripper_hold_force', anonymous=False)

#     # Parameters:
#     # target_width: desired gripper width (0.0 means fully closed)
#     # max_effort: the force in Newtons to hold the object
#     target_width = rospy.get_param('~target_width', 0.03)
#     max_effort   = rospy.get_param('~max_effort', 40.0)

#     # Use the correct action server name based on your topic list.
#     action_server_name = rospy.get_param('~action_server', 'franka_gripper/gripper_action')

#     rospy.loginfo("Waiting for gripper action server [%s]...", action_server_name)
#     client = actionlib.SimpleActionClient(action_server_name, GripperCommandAction)

#     # Wait up to 10 seconds for the server to come up.
#     if not client.wait_for_server(rospy.Duration(10)):
#         rospy.logerr("Could not connect to gripper action server [%s]. "
#                      "Ensure that the franka_gripper node is running.", action_server_name)
#         return

#     rospy.loginfo("Connected to gripper action server [%s].", action_server_name)

#     # Create and populate the goal message.
#     goal = GripperCommandGoal()
#     goal.command.position = target_width   # Command the gripper to be fully closed (or adjust as needed)
#     goal.command.max_effort = max_effort     # Apply this force when holding the object

#     rospy.loginfo("Sending gripper command: target_width=%.3f, max_effort=%.3f",
#                   target_width, max_effort)
#     client.send_goal(goal)
#     rospy.loginfo("Gripper command sent. The gripper should now hold the object continuously.")

#     # Keep the node alive so that the goal remains active.
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
