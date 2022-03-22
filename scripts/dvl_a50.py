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
        self.declare_parameter('ip_address', '192.168.194.95')
        self.my_param = self.get_parameter('ip_address').get_parameter_value().string_value
        self.get_logger().info('IP_ADDRESS: %s' % self.my_param)
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
        self.get_logger().info("Connecting...")
        self.connect()



    #Destructor
    def __del__(self): 
        print("Exiting...")
        self.sock.close()

	#SOCKET
    def connect(self):
        try:
            server_address = (self.my_param, 16171)
            self.sock.connect(server_address)
            self.sock.settimeout(1)
            self.get_logger().info("Socket is connected")
        except socket.error as err:
            self.get_logger().info("No route to host, DVL might be booting? {}".format(err))
            sleep(1)
            self.connect()

    def getData(self):
        raw_data = ""
        data = ""

        while not '\n' in raw_data:
            try:
                rec = self.sock.recv(1) # Add timeout for that
                data = str(rec, 'utf-8')
                if len(rec) == 0:
                    self.get_logger().info("Socket closed by the DVL, reopening")
                    self.connect()
                    continue
                else:
                    raw_data = raw_data + data

            except socket.timeout as err:
                self.get_logger().info("Lost connection with the DVL, reinitiating the connection: {}".format(err))
                self.connect()
                continue
    

        #raw_data = self.oldJson + raw_data
        #self.oldJson = ""
        #raw_data = raw_data.split('\n')
        #self.oldJson = raw_data[1]
        #raw_data = raw_data[0]
        #self.get_logger().info("Data: {}".format(raw_data))
        return raw_data

    #ROS
    def timer_callback(self):
        self.stamp = self.get_clock().now().to_msg()
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        raw_data = self.getData()
        data = json.loads(raw_data)

        self.publish_data(data)
	
    def publish_data(self, data):

        theDVL.header.stamp = self.stamp
        theDVL.header.frame_id = "dvl_50_link"

        if 'time' in data:
            theDVL.time = float(data["time"])
            theDVL.velocity.x = float(data["vx"])
            theDVL.velocity.y = float(data["vy"])
            theDVL.velocity.z = float(data["vz"])
            theDVL.fom = float(data["fom"])
            self.current_altitude = float(data["altitude"])
            theDVL.velocity_valid = data["velocity_valid"]
            
            if self.current_altitude >= 0.0 && theDVL.velocity_valid:
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
