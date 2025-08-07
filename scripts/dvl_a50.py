#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import socket
import json

import time
from time import sleep
from std_msgs.msg import String

from dvl_msgs.msg import DVL
from dvl_msgs.msg import DVLBeam
from dvl_msgs.msg import DVLDR

import select



theDVL = DVL()
beam0 = DVLBeam()
beam1 = DVLBeam()
beam2 = DVLBeam()
beam3 = DVLBeam()

DVLDeadReckoning = DVLDR()


class DVL_A50(Node):
    #Constructor
    def __init__(self):
        super().__init__('dvl_a50_node')

        # device IP
        self.declare_parameter('dvl_address', '192.168.1.99')
        self.device_ip = self.get_parameter('dvl_address').get_parameter_value().string_value

        # local (ROS) IP
        self.declare_parameter('client_address', '192.168.1.181')
        self.client_address = self.get_parameter('client_address').get_parameter_value().string_value

        # DVL port
        self.declare_parameter('port', 16171)
        self.dvl_port = self.get_parameter('port').get_parameter_value().integer_value

        self.get_logger().info(f"Device IP: {self.device_ip}, ROS IP: {self.client_address}, Port: {self.dvl_port}")

        self.dvl_publisher_ = self.create_publisher(DVL, '/dvl/data', 10)
        self.dvl_publisher_pos = self.create_publisher(DVLDR, '/dvl/position', 10)
        timer_period = 0.05  # seconds -> 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.stamp = self.get_clock().now().to_msg()
        self.data = None
        self.oldJson = ""
        self.current_altitude = 0.0
        self.old_altitude = 0.0


        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setblocking(False)
        self._recv_buffer = ""

        self.get_logger().info("Connecting...")
        self.connect()



    #Destructor
    def __del__(self): 
        print("Exiting...")
        self.sock.close()

	#SOCKET
    def connect(self):
        """Try to open a fresh socket to the DVL, retrying until it succeeds."""
        while True:
            try:
                # Close any existing socket
                try:
                    self.sock.close()
                except Exception:
                    pass

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.bind((self.client_address, 0))

                server_address = (self.device_ip, self.dvl_port)
                self.sock.settimeout(1.0)
                self.sock.connect(server_address)

                self.get_logger().info(
                    f"Socket bound to {self.client_address} and connected to {self.device_ip}:{self.dvl_port}"
                )
                return  

            except socket.timeout as err:
                self.get_logger().warn(f"Connect timed out ({err}), retrying…")
                time.sleep(1)

            except socket.error as err:
                self.get_logger().warn(f"Socket error ({err}); retrying…")
                time.sleep(1)


    def getData(self) -> str | None:
        """
        Read available bytes in one chunk, buffer until newline,
        return the next complete line (without the trailing '\n'),
        or None if no full line is ready yet.
        """
        try:
            chunk = self.sock.recv(4096).decode('utf-8', errors='ignore')
            if not chunk:
                # remote closed the socket
                self.get_logger().warn("Socket closed by DVL; reconnecting…")
                self.connect()
                return None
            self._recv_buffer += chunk
        except BlockingIOError:
            # no data available right now
            return None
        except socket.error as e:
            self.get_logger().warn(f"Socket error ({e}); reconnecting…")
            self.connect()
            return None

        if '\n' in self._recv_buffer:
            line, _sep, rest = self._recv_buffer.partition('\n')
            self._recv_buffer = rest
            return line

        return None


    def timer_callback(self):
        self.stamp = self.get_clock().now().to_msg()
        raw_line = self.getData()
        if raw_line is None:
            return  # no complete message yet

        try:
            data = json.loads(raw_line)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parse error: {e}; line: {raw_line}")
            return

        self.publish_data(data)
	
    def publish_data(self, data):

        now = self.get_clock().now().to_msg()

        theDVL.header.stamp = now
        theDVL.header.frame_id = "dvl_50_link"

        if 'time' in data:
            theDVL.time = float(data["time"])
            theDVL.velocity.x = float(data["vx"])
            theDVL.velocity.y = float(data["vy"])
            theDVL.velocity.z = float(data["vz"])
            theDVL.fom = float(data["fom"])
            self.current_altitude = float(data["altitude"])
            theDVL.velocity_valid = data["velocity_valid"]
            
            if self.current_altitude >= 0.0 and theDVL.velocity_valid:
                theDVL.altitude = self.current_altitude
                self.old_altitude = self.current_altitude
            else:
                theDVL.altitude = self.old_altitude


            theDVL.status = data["status"]
            theDVL.form = data["format"]
		
            beam0.id = data["transducers"][0]["id"]
            beam0.velocity = float(data["transducers"][0]["velocity"])
            beam0.distance = float(data["transducers"][0]["distance"])
            beam0.rssi = float(data["transducers"][0]["rssi"])
            beam0.nsd = float(data["transducers"][0]["nsd"])
            beam0.valid = data["transducers"][0]["beam_valid"]
		
            beam1.id = data["transducers"][1]["id"]
            beam1.velocity = float(data["transducers"][1]["velocity"])
            beam1.distance = float(data["transducers"][1]["distance"])
            beam1.rssi = float(data["transducers"][1]["rssi"])
            beam1.nsd = float(data["transducers"][1]["nsd"])
            beam1.valid = data["transducers"][1]["beam_valid"]
		
            beam2.id = data["transducers"][2]["id"]
            beam2.velocity = float(data["transducers"][2]["velocity"])
            beam2.distance = float(data["transducers"][2]["distance"])
            beam2.rssi = float(data["transducers"][2]["rssi"])
            beam2.nsd = float(data["transducers"][2]["nsd"])
            beam2.valid = data["transducers"][2]["beam_valid"]
		
            beam3.id = data["transducers"][3]["id"]
            beam3.velocity = float(data["transducers"][3]["velocity"])
            beam3.distance = float(data["transducers"][3]["distance"])
            beam3.rssi = float(data["transducers"][3]["rssi"])
            beam3.nsd = float(data["transducers"][3]["nsd"])
            beam3.valid = data["transducers"][3]["beam_valid"]
		
            theDVL.beams = [beam0, beam1, beam2, beam3]
		
            self.dvl_publisher_.publish(theDVL)
            
        if 'ts' in data:

            DVLDeadReckoning.header.stamp = now
            DVLDeadReckoning.header.frame_id = "dvl_50_link"

            DVLDeadReckoning.time = float(data["ts"])
            DVLDeadReckoning.position.x = float(data["x"])
            DVLDeadReckoning.position.y = float(data["y"])
            DVLDeadReckoning.position.z = float(data["z"])
            DVLDeadReckoning.pos_std = float(data["std"])
            DVLDeadReckoning.roll = float(data["roll"])
            DVLDeadReckoning.pitch = float(data["pitch"])
            DVLDeadReckoning.yaw = float(data["yaw"])
            DVLDeadReckoning.type = data["type"]
            DVLDeadReckoning.status = data["status"]
            DVLDeadReckoning.format = data["format"]
            
            self.dvl_publisher_pos.publish(DVLDeadReckoning)


def main(args=None):
    rclpy.init(args=args)

    dvl_a50 = DVL_A50()
    rclpy.spin(dvl_a50)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    dvl_a50.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
