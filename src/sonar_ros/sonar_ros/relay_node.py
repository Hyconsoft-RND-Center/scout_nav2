#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('relay_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.get_logger().info('Ready to receive commands via topic (RON to switch ON, ROFF to switch OFF)')

        # 토픽 구독
        self.command_subscriber = self.create_subscription(
            String,
            'relay_command',
            self.command_callback,
            10)
        
        self.current_command = None

    def command_callback(self, msg):
        self.current_command = msg.data.strip()
        if self.current_command == 'ON':
            self.serial_port.write(b'DO\n')
        elif self.current_command == 'OFF':
            self.serial_port.write(b'DF\n')
        else:
            self.get_logger().warning('Invalid command received: {}'.format(self.current_command))

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        serial_node.get_logger().info('Exiting...')
    finally:
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# ON 명령 발행
# ros2 topic pub /relay_command std_msgs/String "data: 'ON'"

# OFF 명령 발행
# ros2 topic pub /relay_command std_msgs/String "data: 'OFF'"
