from robotic_sol_interfaces.srv import ReadSensor

import rclpy
import socket
import sys
import numpy as np
import time
#from sensor import Sensor

from rclpy.node import Node

# requesting 10 samples on each call
number_of_samples = 10

class SensorMgr(Node):

    def __init__(self):
        super().__init__('sensor_mgr_node')
        
        self.sensor_sample_rate = 2000

        # Create a wall timer
        sensor_timer = self.create_timer(1/self.sensor_sample_rate, self.timer_cb)

        # Create a service
        self.srv = self.create_service(ReadSensor, 'srv/read_sensor', self.read_sensor_cb)

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address = ('127.0.0.3', 10000)
        print('connecting to {} port {}'.format(*server_address))
        self.sock.connect(server_address)

    def timer_cb(self):        
        # self.get_logger().info("Wall timer rang")        
        message_string = str(number_of_samples)
        message = message_string.encode()
        self.sock.sendall(message)

        byte_data = self.sock.recv(10000)
        data = np.frombuffer(byte_data)
        # print("Data: ", len(data), data)

    def read_sensor_cb(self, request, response):
        self.get_logger().info('Incoming request: sensor %d' % (request.sensor_num)) 
        
        response.sensor_value[0] = 0.0
        response.sensor_value[1] = 1.0
        response.sensor_value[2] = 2.0

        return response

def main(args=None):
    print('Starting sensor manager')
    rclpy.init(args=args)

    # node = rclpy.create_node("robotic_sol_node")
    node = SensorMgr()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
