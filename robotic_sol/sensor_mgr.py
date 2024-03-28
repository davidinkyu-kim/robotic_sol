from robotic_sol_interfaces.srv import ReadSensor

import rclpy
import socket
import sys
import numpy as np
import time

from rclpy.node import Node
from functools import partial
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# requesting 10 samples on each call
number_of_samples = 10

class Sensor3dof:
    def __init__(self, name, ip_address, port, sample_rate):
        print(f"Initializing sensor: {name}")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.name = name
        self.ip_address = ip_address
        self.port = port
        self.sample_rate = sample_rate

        self.request_cnt = 0

        message_string = str(number_of_samples)
        self.request_message = message_string.encode()

        server_address = (self.ip_address, self.port)
        self.sock.connect(server_address)
        print(f"Connected {name} at {ip_address}:{port}")

class SensorMgr(Node):

    def __init__(self):
        super().__init__('sensor_mgr_node')
        self.package_name = 'robotic_sol'

        self.sensors = [
            Sensor3dof('sensor1', '127.0.0.3', 10000, 2000),
            Sensor3dof('sensor2', '127.0.0.1', 10000, 4000)
        ]

        for sensor in self.sensors:            
            # Create a timer per sensor
            sensor.timer_cb_group = ReentrantCallbackGroup()            
            sensor.timer = self.create_timer(1/sensor.sample_rate, partial(self.timer_cb, arg=sensor.name), callback_group=sensor.timer_cb_group)
            self.get_logger().info(f"Created a timer for {sensor.name} at rate {sensor.sample_rate}hz")
            
            # Create a service per sensor
            sensor.read_srv_ = self.create_service(ReadSensor, self.package_name+'/'+sensor.name+'/srv/read_sensor', partial(self.read_sensor_cb, arg=sensor.name) )
            self.get_logger().info("Created a service for %s" % sensor.name)

    def timer_cb(self, arg):        
        self.get_logger().debug(f"Wall timer rang from sensor: {arg}") 
        for sensor in self.sensors:
            if sensor.name is arg:
                sensor.request_cnt += 1
                self.get_logger().debug(f"Sending request to {sensor.name}, {sensor.ip_address} cnt: {sensor.request_cnt}")
                sensor.sock.sendall(sensor.request_message)

                byte_data = sensor.sock.recv(10000)                
                sensor.buffered_data = np.frombuffer(byte_data)
                sensor.latest_data = sensor.buffered_data[-3:]            

    def read_sensor_cb(self, request, response, arg):
        self.get_logger().debug(f"Incoming service request for sensor: {arg}") 
                
        for sensor in self.sensors:
            if sensor.name is arg:                
                self.get_logger().debug(f"Responding data for {sensor.name}")
                response.sensor_value = sensor.latest_data

        return response

def main(args=None):
    print('Starting sensor manager')
    rclpy.init(args=args)

    node = SensorMgr()

    while rclpy.ok():
        rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
