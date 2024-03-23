from robotic_sol_interfaces.srv import ReadSensor
from robotic_sol_interfaces.msg import SensorsOutput

import rclpy
import numpy as np
import time

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# requesting 10 samples on each call
number_of_samples = 10

class SensorClient(Node):

    def __init__(self):
        super().__init__('sensor_client_node')
        
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.sensor_sample_rate = 1000
        self.sensor_publish_rate = 500

        # Create a wall timer
        self.sersor_read_timer_ = self.create_timer(1/self.sensor_sample_rate, self.sensor_read_timer_cb, callback_group=timer_cb_group)

        # Create a service client
        self.read_sensor_srv_cli_ = self.create_client(ReadSensor, 'srv/read_sensor', callback_group=client_cb_group)
        while not self.read_sensor_srv_cli_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again')        

    def sensor_read_timer_cb(self):        
        self.get_logger().info("Wall timer rang")        
        req = ReadSensor.Request()
        req.sensor_num = 0
        self.get_logger().info('Sending request')
        self.read_sensor_srv_cli_.call(req)
        self.get_logger().info('Received response')


def main(args=None):
    print('Starting sensor client')
    rclpy.init(args=args)
    
    node = SensorClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
