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

class SensorClass():
    def __init__(self, name):
        print(f"Initializing sensor: {name}")        
        self.name = name
        self.sensor_msg = Sensor3dof()
        self.data_available = False     # Flag for at least one data point heard from the sensor        

class SensorClient(Node):

    def __init__(self):
        super().__init__('sensor_client_node')
        self.package_name = 'robotic_sol'

        self.sensors = [
            SensorClass("sensor1"),
            SensorClass("sensor2")
        ]

        timer_cb_group = ReentrantCallbackGroup()        
        
        self.sensor_publish_rate = 500

        # Create a wall timer
        self.process_timer_ = self.create_timer(1/self.sensor_publish_rate, self.process_timer_cb, callback_group=timer_cb_group)

        # Create a publisher
        self.sensor_pub_ = self.create_publisher(SensorsOutput, self.package_name+'/msg/sensors_output', 10)
        
        # Create a service client per sensor
        for sensor in self.sensors:
            sensor.read_sensor_srv_cli_ = self.create_client(ReadSensor, self.package_name+'/'+sensor.name+'/srv/read_sensor', callback_group = timer_cb_group)
            while not sensor.read_sensor_srv_cli_.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting again')
            self.get_logger().info(f"Connected to the {sensor.name} read sensor service")
            
            sensor.did_request = False
            sensor.data = np.empty(len(ReadSensor.Response().sensor_value))

    def process_timer_cb(self):        
        self.get_logger().debug("Wall timer rang from timer callback")        
        
        output_msg = SensorsOutput()

        self.all_sensors_available = True

        for sensor in self.sensors:
            # Invoke a service call if did not
            if not sensor.did_request:
                sensor.did_request = True        
                req = ReadSensor.Request()                
                self.get_logger().debug(f"Sending request to {sensor.name}")
                sensor.future = sensor.read_sensor_srv_cli_.call_async(req) # Async call to retrieve result later when available
                
            # Read in if service call received response
            else:
                self.get_logger().debug(f"{sensor.name} requested and waiting for response")
                if sensor.future.done():
                    result = sensor.future.result()
                    self.get_logger().debug('Received response %f %f %f' % (result.sensor_value[0], result.sensor_value[1], result.sensor_value[2]))                    
                    sensor.data = result.sensor_value

                    sensor.data_available = True                     
                    sensor.did_request = False # Reset flag to request data again

            self.all_sensors_available = self.all_sensors_available and sensor.data_available

            if self.all_sensors_available:
                # Populate latest data into message
                sensor.sensor_msg.sensor_value = sensor.data

                # Populate the topic
                output_msg.sensors.append(sensor.sensor_msg)

        # Publish the latest sensor msgs
        if self.all_sensors_available:
            self.sensor_pub_.publish(output_msg)
            self.get_logger().debug("Published the data")

def main(args=None):
    print('Starting sensor client')
    rclpy.init(args=args)
    
    node = SensorClient()

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
