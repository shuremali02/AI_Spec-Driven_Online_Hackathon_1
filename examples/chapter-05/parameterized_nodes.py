import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('robot_name', 'default_robot', 'Name of the robot')
        self.declare_parameter('max_velocity', 1.0, 'Maximum linear velocity')
        self.declare_parameter('wheel_diameter', 0.1, 'Wheel diameter in meters')
        self.declare_parameter('sensors_enabled', [True, True, False, True], 'Array of sensor enable flags')
        self.declare_parameter('control_mode', 'velocity', 'Control mode: velocity, position, or effort')
        self.declare_parameter('debug_mode', False, 'Enable debug output')

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.sensors_enabled = self.get_parameter('sensors_enabled').value
        self.control_mode = self.get_parameter('control_mode').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Log parameter values
        self.get_logger().info(f'Robot Name: {self.robot_name}')
        self.get_logger().info(f'Max Velocity: {self.max_velocity}')
        self.get_logger().info(f'Wheel Diameter: {self.wheel_diameter}')
        self.get_logger().info(f'Sensors Enabled: {self.sensors_enabled}')
        self.get_logger().info(f'Control Mode: {self.control_mode}')
        self.get_logger().info(f'Debug Mode: {self.debug_mode}')

        # Example of how to handle parameter changes at runtime
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Callback for handling parameter changes at runtime."""
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
                self.get_logger().info(f'Robot name changed to: {self.robot_name}')
            elif param.name == 'max_velocity':
                self.max_velocity = param.value
                self.get_logger().info(f'Max velocity changed to: {self.max_velocity}')
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                if self.debug_mode:
                    self.get_logger().info('Debug mode enabled')
                else:
                    self.get_logger().info('Debug mode disabled')

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterizedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()