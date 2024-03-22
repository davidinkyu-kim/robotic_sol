from robotic_sol_interfaces.srv import ReadSensor

import rclpy
from rclpy.node import Node

class RoboticSol(Node):

    def __init__(self):
        super().__init__('robotic_sol')
        self.srv = self.create_service(ReadSensor, 'srv/read_sensor', self.read_sensor_callback)

    def read_sensor_callback(self, request, response):
        self.get_logger().info('Incoming request\n: %d' % (request.sensor_num)) # CHANGE

        response.sensor_values[0] = 0.0
        response.sensor_values[1] = 1.0
        response.sensor_values[2] = 2.0

        return response

def main(args=None):
    print('Hi from robotic_sol.')
    rclpy.init(args=args)

    # node = rclpy.create_node("robotic_sol_node")
    node = RoboticSol()

    while rclpy.ok():
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
