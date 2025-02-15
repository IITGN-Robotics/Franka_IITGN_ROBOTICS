

#!/usr/bin/env python

import rospy
import serial
import threading
from sensor_msgs.msg import JointState

class JointStateLogger(object):
    def __init__(self):
        # Get CSV file path from parameter (change the default as needed)
        self.file_path = rospy.get_param('~csv_file_path',
                                         '/home/iitgn-robotics/franka_ros_ws/src/franka_ros/franka_iitgn/data/joint_data.csv')
        try:
            self.csv_file = open(self.file_path, 'w')
        except Exception as e:
            rospy.logerr("Failed to open CSV file: %s. Error: %s", self.file_path, str(e))
            raise e

        rospy.loginfo("Opened CSV file: %s", self.file_path)
        self.header_written = False

        # Variables to hold the latest Arduino sensor data and its reception time.
        self.latest_arduino_data = None
        self.latest_arduino_time = None

        # Define a synchronization threshold (here, 50 ms).
        self.sync_threshold = rospy.Duration(0.05)

        # Set up the serial connection to the Arduino.
        try:
            # Change the device path to '/dev/ttyUSB0' if that is your Arduino's connection.
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        except Exception as e:
            rospy.logerr("Failed to open serial port: %s", str(e))
            raise e

        # Start a thread to continuously read from the Arduino serial port.
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True  # Ensures the thread exits when the main program exits.
        self.serial_thread.start()

        # Subscribe to the joint state topic.
        self.joint_sub = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.joint_state_callback)

    def read_serial(self):
        """
        Continuously read lines from the Arduino's serial port.
        Each valid line should be a CSV string with 5 comma-separated values:
        AcX,AcY,AcZ,Force,ContactMic,ContactMic_gt
        """
        while not rospy.is_shutdown():
            try:
                # Read a line from the serial port and decode it.
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    rospy.loginfo("Received from Arduino: %s", line)
                    # Parse the CSV string.
                    data = [x.strip() for x in line.split(',')]
                    if len(data) != 6:
                        rospy.logwarn("Expected 6 values from Arduino, got %d: %s", len(data), data)
                    else:
                        # Update the latest Arduino data and record the current ROS time.
                        self.latest_arduino_data = data
                        self.latest_arduino_time = rospy.Time.now()
            except Exception as e:
                rospy.logerr("Error reading from serial port: %s", str(e))

    def joint_state_callback(self, msg):
        # Write the CSV header on the first joint state callback.
        if not self.header_written:
            header = ['time']
            # Add joint positions.
            for name in msg.name:
                header.append(name + '_pos')
            # Add joint velocities.
            for name in msg.name:
                header.append(name + '_vel')
            # Add joint efforts.
            for name in msg.name:
                header.append(name + '_eff')
            # Append headers for Arduino sensor data (9 columns).
            header.extend(['flexpin1', 'flexpin2', 'flexpin3', 'Force', 'ContactMic', 'ContactMic_gt'])
            self.csv_file.write(','.join(header) + "\n")
            self.header_written = True

        # Use the joint state's header timestamp if available; otherwise, use the current time.
        if msg.header.stamp != rospy.Time():
            joint_time = msg.header.stamp
        else:
            joint_time = rospy.Time.now()
        t = joint_time.to_sec()

        # Check if the latest Arduino data is recent enough.
        if (self.latest_arduino_time is not None and 
            abs(joint_time - self.latest_arduino_time) < self.sync_threshold):
            arduino_data = self.latest_arduino_data
        else:
            rospy.logwarn("Arduino data not synchronized. Joint time: %s, Arduino time: %s",
                          str(joint_time), str(self.latest_arduino_time))
            arduino_data = [''] * 9  # Fill with empty strings if data is stale.

        # Prepare the CSV row.
        row = [str(t)]
        row.extend([str(pos) for pos in msg.position])
        row.extend([str(vel) for vel in msg.velocity])
        row.extend([str(eff) for eff in msg.effort])
        row.extend(arduino_data)
        self.csv_file.write(','.join(row) + "\n")
        self.csv_file.flush()

def main():
    rospy.init_node('joint_state_logger', anonymous=False)
    logger = JointStateLogger()
    rospy.spin()
    # Close the CSV file on shutdown.
    logger.csv_file.close()

if __name__ == '__main__':
    main()



#!/usr/bin/env python

# import rospy
# import serial
# import threading
# from sensor_msgs.msg import JointState

# class JointStateLogger(object):
#     def __init__(self):
#         # Get CSV file path from parameter (change the default as needed)
#         self.file_path = rospy.get_param('~csv_file_path',
#                                          '/home/iitgn-robotics/franka_ros_ws/src/franka_ros/franka_iitgn/data/joint_data.csv')
#         try:
#             self.csv_file = open(self.file_path, 'w')
#         except Exception as e:
#             rospy.logerr("Failed to open CSV file: %s. Error: %s", self.file_path, str(e))
#             raise e

#         rospy.loginfo("Opened CSV file: %s", self.file_path)
#         self.header_written = False

#         # Variables to hold the latest sensor data and its reception time for each Arduino.
#         self.latest_arduino1_data = None
#         self.latest_arduino1_time = None

#         self.latest_arduino2_data = None
#         self.latest_arduino2_time = None

#         # Define a synchronization threshold (here, 50 ms).
#         self.sync_threshold = rospy.Duration(0.05)

#         # Set up the serial connection for Arduino board 1 (/dev/ttyUSB0)
#         try:
#             self.serial_port1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#             rospy.loginfo("Opened serial port /dev/ttyUSB0 for board 1")
#         except Exception as e:
#             rospy.logerr("Failed to open serial port /dev/ttyUSB0: %s", str(e))
#             raise e

#         # Set up the serial connection for Arduino board 2 (/dev/ttyACM0)
#         try:
#             self.serial_port2 = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#             rospy.loginfo("Opened serial port /dev/ttyACM0 for board 2")
#         except Exception as e:
#             rospy.logerr("Failed to open serial port /dev/ttyACM0: %s", str(e))
#             raise e

#         # Start threads to continuously read from both Arduino serial ports.
#         self.serial_thread1 = threading.Thread(target=self.read_serial, args=(self.serial_port1, 1))
#         self.serial_thread1.daemon = True  # Ensures the thread exits when the main program exits.
#         self.serial_thread1.start()

#         self.serial_thread2 = threading.Thread(target=self.read_serial, args=(self.serial_port2, 2))
#         self.serial_thread2.daemon = True
#         self.serial_thread2.start()

#         # Subscribe to the joint state topic.
#         self.joint_sub = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.joint_state_callback)

#     def read_serial(self, port, board_id):
#         """
#         Continuously read lines from an Arduino serial port.
#         For board 1 (USB0), expect a CSV string with 5 comma-separated values.
#         For board 2 (ACM0), expect a CSV string with 1 comma-separated value.
#         """
#         while not rospy.is_shutdown():
#             try:
#                 # Read a line from the serial port and decode it.
#                 line = port.readline().decode('utf-8').strip()
#                 if line:
#                     rospy.loginfo("Received from Arduino board %d: %s", board_id, line)
#                     # Parse the CSV string.
#                     data = [x.strip() for x in line.split(',')]
#                     expected_length = 5 if board_id == 1 else 1
#                     if len(data) != expected_length:
#                         rospy.logwarn("Expected %d values from Arduino board %d, got %d: %s", 
#                                       expected_length, board_id, len(data), data)
#                     else:
#                         # Update the latest sensor data and record the current ROS time.
#                         current_time = rospy.Time.now()
#                         if board_id == 1:
#                             self.latest_arduino1_data = data
#                             self.latest_arduino1_time = current_time
#                         else:
#                             self.latest_arduino2_data = data
#                             self.latest_arduino2_time = current_time
#             except Exception as e:
#                 rospy.logerr("Error reading from serial port board %d: %s", board_id, str(e))

#     def joint_state_callback(self, msg):
#         # Write the CSV header on the first joint state callback.
#         if not self.header_written:
#             header = ['time']
#             # Add joint positions.
#             for name in msg.name:
#                 header.append(name + '_pos')
#             # Add joint velocities.
#             for name in msg.name:
#                 header.append(name + '_vel')
#             # Add joint efforts.
#             for name in msg.name:
#                 header.append(name + '_eff')
#             # Append headers for Arduino sensor data from both boards.
#             # Board 1 (USB0): 5 values.
#             header.extend([
#                 'board1_AcX', 'board1_AcY', 'board1_AcZ', 'board1_Force', 'board1_ContactMic'
#             ])
#             # Board 2 (ACM0): 1 value.
#             header.extend(['board2_value'])
#             self.csv_file.write(','.join(header) + "\n")
#             self.header_written = True

#         # Use the joint state's header timestamp if available; otherwise, use the current time.
#         if msg.header.stamp != rospy.Time():
#             joint_time = msg.header.stamp
#         else:
#             joint_time = rospy.Time.now()
#         t = joint_time.to_sec()

#         # Retrieve board 1 data if synchronized.
#         if (self.latest_arduino1_time is not None and 
#             abs(joint_time - self.latest_arduino1_time) < self.sync_threshold):
#             board1_data = self.latest_arduino1_data
#         else:
#             rospy.logwarn("Board 1 data not synchronized. Joint time: %s, Board1 time: %s",
#                           str(joint_time), str(self.latest_arduino1_time))
#             board1_data = [''] * 5  # Fill with empty strings if data is stale.

#         # Retrieve board 2 data if synchronized.
#         if (self.latest_arduino2_time is not None and 
#             abs(joint_time - self.latest_arduino2_time) < self.sync_threshold):
#             board2_data = self.latest_arduino2_data
#         else:
#             rospy.logwarn("Board 2 data not synchronized. Joint time: %s, Board2 time: %s",
#                           str(joint_time), str(self.latest_arduino2_time))
#             board2_data = [''] * 1  # Fill with empty strings if data is stale.

#         # Prepare the CSV row.
#         row = [str(t)]
#         row.extend([str(pos) for pos in msg.position])
#         row.extend([str(vel) for vel in msg.velocity])
#         row.extend([str(eff) for eff in msg.effort])
#         row.extend(board1_data)
#         row.extend(board2_data)
#         self.csv_file.write(','.join(row) + "\n")
#         self.csv_file.flush()

# def main():
#     rospy.init_node('joint_state_logger', anonymous=False)
#     logger = JointStateLogger()
#     rospy.spin()
#     # Close the CSV file on shutdown.
#     logger.csv_file.close()

# if __name__ == '__main__':
#     main()
