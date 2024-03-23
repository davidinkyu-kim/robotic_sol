from robotic_sol_interfaces.srv import ReadSensor
from robotic_sol_interfaces.msg import Sensor3dof, SensorsOutput

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
        
        cb_group = ReentrantCallbackGroup()
        # client_cb_group = ReentrantCallbackGroup()
        # timer_cb_group = MutuallyExclusiveCallbackGroup()

        # self.sensor_sample_rate = 500
        self.sensor_publish_rate = 500

        # Create a wall timer
        self.sensor_read_timer_ = self.create_timer(1/self.sensor_publish_rate, self.sensor_read_timer_cb, callback_group=cb_group)

        # Create a publisher
        self.sensor_pub_ = self.create_publisher(SensorsOutput, '/msg/sensors_output', 10)

        # Create a service client
        # self.read_sensor_srv_cli_ = self.create_client(ReadSensor, 'srv/read_sensor', callback_group=client_cb_group)
        self.read_sensor_srv_cli_ = self.create_client(ReadSensor, 'srv/read_sensor', callback_group = cb_group)
        while not self.read_sensor_srv_cli_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again')

        self.did_run = False
        self.data = [0,0,0]        

    def sensor_read_timer_cb(self):        
        self.get_logger().info("Wall timer rang from timer callback")        
        
        # Invoke service call if did not
        if not self.did_run:
            self.did_run = True        
            req = ReadSensor.Request()
            req.sensor_num = 0
            self.get_logger().info('Sending request')
            self.future = self.read_sensor_srv_cli_.call_async(req)
            
        if self.future.done():
            self.result = self.future.result()
            self.get_logger().info('Received response %d %d %d' % (self.result.sensor_value[0], self.result.sensor_value[1], self.result.sensor_value[2]))                    
            self.data[0] = self.result.sensor_value[0]
            self.data[1] = self.result.sensor_value[1]
            self.data[2] = self.result.sensor_value[2]
            self.did_run = False

        # Publish existing data
        sensor_msg = Sensor3dof()
        sensor_msg.sensor_value[0] = self.data[0]
        sensor_msg.sensor_value[1] = self.data[1]
        sensor_msg.sensor_value[2] = self.data[2]

        output_msg = SensorsOutput()
        output_msg.sensors.append(sensor_msg)

        self.sensor_pub_.publish(output_msg)
        self.get_logger().info("Publishing the data")
        
        # result = self.future.Result()
        # print("Result",result.sensor_value[0])

def main(args=None):
    print('Starting sensor client')
    rclpy.init(args=args)
    
    node = SensorClient()

    while rclpy.ok():
        # if node.did_run:
        #     if not node.did_get_result:
        #         # node.get_logger().info("Skipping a service call as we are waiting for the result")
        #         # node.sensor_read_timer_.cancel()
        #     else:
        #         node.get_logger().info("Got the result back from the server, resetting ")
        #         node.did_run = False
        #         node.did_get_result = False
        #         node.sensor_read_timer_.reset()        
        
        rclpy.spin_once(node)

    # executor = MultiThreadedExecutor()
    # executor.add_node(node)

    # try:
    #     node.get_logger().info('Beginning client, shut down with CTRL-C')
    #     executor.spin()
    # except KeyboardInterrupt:
    #     node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
