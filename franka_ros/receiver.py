#!/usr/bin/env python3

import struct
import pickle
import socket
import numpy as np

def receive_array():
    host = '10.7.40.192'  # Change to sender's IP if running on different machines  
    port = 12345         # Port to connect to

    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   
    try:
        client_socket.connect((host, port))
        print("Connected to server:", host, port)

        while True:
            # Receive the first 4 bytes (data length)
            raw_data_len = client_socket.recv(4)
            if not raw_data_len:
                print("No data received, closing connection.")
                break

            # Unpack the length
            data_len = struct.unpack('>I', raw_data_len)[0]  # '>I' means big-endian unsigned int
           
            # Receive the actual data
            data = b""
            while len(data) < data_len:
                packet = client_socket.recv(min(4096, data_len - len(data)))  # Receive in chunks
                if not packet:  # Connection closed
                    print("Connection closed while receiving data.")
                    break
                data += packet

            if len(data) == data_len:  # Ensure full data received
                array = pickle.loads(data)
                if isinstance(array, np.ndarray) and array.shape == (4, 4):  
                    print("Received 4x4 transformation matrix:\n", array)
                else:
                    print("Received data (not a 4x4 matrix):", array)
            else:
                print(f"Data length mismatch: received {len(data)} bytes, expected {data_len}.")

    except (ConnectionResetError, OSError) as e:
        print(f"Connection error: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user, closing connection...")
    finally:
        client_socket.close()  # Close the client socket
        print("Socket closed.")

if __name__ == "__main__":
    receive_array()