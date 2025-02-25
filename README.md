#Franka ROS Control, IITGN Robotics Lab

This repository contains ROS commands and scripts for controlling the Franka Emika Panda robot. The following sections outline the installation guide, robot setup, and various control commands for trajectory planning, gripper control, and more.

---

## 1. Installation Guide

### 1.1 Key Points to Remember

- **Compatibility Check:** Ensure that the versions of `libfranka` and `franka_ros` are compatible. Refer to the [Franka Emika Compatibility Documentation](https://frankaemika.github.io/docs/compatibility.html) for details.

- **Install the Real-Time Kernel (PREEMPT Version):**
  - For **ROS1**, follow the [Franka Emika ROS Installation Guide for Linux](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).
  - For **ROS2**, use the instructions for setting up real-time kernels with Ubuntu Pro: [Ubuntu Real-Time Guide](https://ubuntu.com/real-time).
  
  **Important:** Always use `sudo` with commands to install the real-time kernel to avoid any errors.

- **Verify Real-Time Setup:** After installation, check that the real-time kernel is properly set up by running the following command:
  
  ```bash
  uname -a
  ```

  During boot, in the **GRUB menu**, navigate to **Advanced Options** and select the real-time kernel patch.

---

## 2. Robot Setup and Activation

### 2.1 Turning on the Robot

1. **Power On:** Turn on the controller for the Franka Emika Panda robot. Wait until the blue LED light starts blinking.
  
2. **Access the Robot via Browser:** Open your web browser and navigate to the robot’s IP address:

   ```
   https://192.168.1.11
   ```

3. **Unlock Joints and Activate FCI:** On the web interface, unlock the robot joints and activate the **FCI** (Franka Control Interface). A **green light** will appear on the FR3 controller.

4. **Ready to Control:** Once the joints are unlocked and FCI is activated, you are ready to start controlling the robot!

---

## 3. Launch Franka Control

To launch the Franka control interface, use:

```bash
roslaunch franka_control franka_control.launch robot_ip:=192.168.1.11 load_gripper:=true robot:=fr3
```

This will load the controller and start the robot.

---

## 4. Example Controllers

### 4.1 Joint Impedance Example Controller

Launch the joint impedance example controller:

```bash
roslaunch franka_example_controllers joint_impedance_example_controller.launch robot_ip:=192.168.1.11 load_gripper:=true robot:=fr3
```

### 4.2 Move to Start Position

Launch the move-to-start controller:

```bash
roslaunch franka_example_controllers move_to_start.launch robot_ip:=192.168.1.11 load_gripper:=true robot:=fr3
```

### 4.3 Cartesian Pose Example Controller

Launch the Cartesian pose controller:

```bash
roslaunch franka_example_controllers cartesian_pose_example_controller.launch robot_ip:=192.168.1.11 load_gripper:=true robot:=fr3
```

---

## 5. Transformation Matrices

### 5.1 Home Position Transformation Matrix

This is the transformation matrix of the robot’s home position:

```
[ 0.999954 -0.00943733  0.00168051  0
 -0.00944125 -0.999953  0.00233788  0
  0.00165837 -0.00235364 -0.999996  0
  0.307535 -0.000656046 0.486955 1 ]
```

---

## 6. Cartesian Pose Control

### 6.1 Continuous X-Direction Movement

To implement continuous movement in the X-direction, use the following code in your Cartesian pose controller:

```cpp
void CartesianPoseExampleController::update(const ros::Time& /* time */, const ros::Duration& period) {
  elapsed_time_ += period;
  std::array<double, 16> current_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  static double delta_x = 0.0;
  delta_x += 0.0001;
  current_pose[12] += delta_x * 0.001;
  cartesian_pose_handle_->setCommand(current_pose);
}
```

---

## 7. Cartesian Impedance Controller for Teleoperation

To teleoperate the Franka robot:

1. Launch the Cartesian Impedance Controller:

   ```bash
   roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=192.168.1.11 load_gripper:=true robot:=fr3
   ```

2. In another terminal, launch the teleoperation node:

   ```bash
   roslaunch joy_franka_control teleop_franka.launch
   ```

3. Hold the **L2 button** on the joystick to enable translation and use the joystick to move the robot.

---

## 8. 6D Pose Estimation and Object Manipulation

For 6D pose estimation and object manipulation, follow the instructions below:

1. Clone and set up the 6D pose estimation repository:
   - GitHub Repository: [live-pose](https://github.com/Kaivalya192/live-pose)

2. Run the `receiver.py` script on your PC to get the transformation matrix of the object in the camera frame using TCP client.

3. Use the `object_on_the_base.py` script to get the transformation of the object with respect to the Franka base frame.

4. Use the Cartesian pose controller to move the robot to the desired position based on the transformation matrix.

5. Update the transformation matrix as needed and refer to `/home/iitgn-robotics/franka_ros_ws/src/franka_ros/franka_iitgn/src/cartesian_controller_front.cpp` for additional guidance.

---

## 9. Joint Trajectory Commands

### 9.1 Basic Joint Trajectory Command

To move the robot using a joint trajectory command, you can publish to the `effort_joint_trajectory_controller` topic:

```bash
rostopic pub /effort_joint_trajectory_controller/command trajectory_msgs/JointTrajectory "..."
```

**Note:** Make sure to change the controller to `effort_joint_trajectory_controller` in `franka_control.launch`.

---

## 10. Gripper Control

### 10.1 Grasp an Object

To command the gripper to grasp an object, use the following command:

```bash
rostopic pub --once /franka_gripper/grasp/goal franka_gripper/GraspActionGoal "goal: {width: 0.05, epsilon:{inner: 0.005, outer: 0.005 }, speed: 0.1, force: 5.0}"
```

### 10.2 Homing Command

To return the gripper to its home position, use:

```bash
rostopic pub /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}"
```

### 10.3 Continuous Gripper Movement

To continuously control the gripper, use the following loop:

```bash
while true; do
  rostopic pub -r 10 /franka_gripper/gripper_action/goal control_msgs/GripperCommandActionGoal "
  {command: {position: 0.08, max_effort: 5.0}}"
  sleep 0.1
done
```

---

## 11. Additional Resources

For more detailed installation and configuration instructions, please refer to the official Franka Emika documentation:

- [Franka Emika Documentation](https://frankaemika.github.io/docs/)

--- 

This README provides a comprehensive guide for setting up and controlling the Franka Emika Panda robot. If you encounter any issues or have further questions, refer to the official documentation or reach out to the community.

