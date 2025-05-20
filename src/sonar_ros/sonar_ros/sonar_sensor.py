#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('sonar_sensor')
        self.declare_parameter('port', '/dev/sonar')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        self.serial_port = serial.Serial(port, baudrate, timeout=1)

        # 각 센서에 대한 퍼블리셔 생성
        self.s1_publisher = self.create_publisher(Range, 'sensor/s1_data', 10)
        self.s2_publisher = self.create_publisher(Range, 'sensor/s2_data', 10)
        self.s3_publisher = self.create_publisher(Range, 'sensor/s3_data', 10)
        self.s4_publisher = self.create_publisher(Range, 'sensor/s4_data', 10)
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.send_request()
        self.get_logger().info('Sent request to Arduino')

    def send_request(self):
        self.serial_port.write(b'SE\n')


    def timer_callback(self):
        self.send_request() 
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f"Received: {line}")

            if line:
                # 데이터를 쉼표로 분리
                data = line.split(',')
                
                if len(data) >= 4:
                    self.publish_sonar_data(data)
                else:
                    self.get_logger().warn("Not enough data received")
            else:
                self.get_logger().warn("No data received")
        else:
            self.get_logger().warn("No data available in serial buffer")

    def publish_sonar_data(self, data):
        # S1 (SR) 센서 데이터 발행
        s1_msg = Range()
        s1_msg.header.frame_id = "S1"
        s1_msg.header.stamp = self.get_clock().now().to_msg()
        s1_msg.radiation_type = 0
        s1_msg.field_of_view = 9.0 / 180.0 * 3.14159  # M_PI
        s1_msg.min_range = 0.01
        s1_msg.max_range = 2.0
        s1_msg.range = ((float(data[0]) * 6.0) - 300.0) / 1000.0
        self.s1_publisher.publish(s1_msg)

        # S2 (SL) 센서 데이터 발행
        s2_msg = Range()
        s2_msg.header.frame_id = "S2"
        s2_msg.header.stamp = self.get_clock().now().to_msg()
        s2_msg.radiation_type = 0
        s2_msg.field_of_view = 9.0 / 180.0 * 3.14159  # M_PI
        s2_msg.min_range = 0.01
        s2_msg.max_range = 2.0
        s2_msg.range = ((float(data[1]) * 6.0) - 300.0) / 1000.0
        self.s2_publisher.publish(s2_msg)

        # S3 (RL) 센서 데이터 발행
        s3_msg = Range()
        s3_msg.header.frame_id = "S3"
        s3_msg.header.stamp = self.get_clock().now().to_msg()
        s3_msg.radiation_type = 0
        s3_msg.field_of_view = 9.0 / 180.0 * 3.14159  # M_PI
        s3_msg.min_range = 0.01
        s3_msg.max_range = 2.0
        s3_msg.range = ((float(data[2]) * 6.0) - 300.0) / 1000.0
        self.s3_publisher.publish(s3_msg)

        # S4 (RR) 센서 데이터 발행
        s4_msg = Range()
        s4_msg.header.frame_id = "S4"
        s4_msg.header.stamp = self.get_clock().now().to_msg()
        s4_msg.radiation_type = 0
        s4_msg.field_of_view = 9.0 / 180.0 * 3.14159  # M_PI
        s4_msg.min_range = 0.01
        s4_msg.max_range = 2.0
        s4_msg.range = ((float(data[3]) * 6.0) - 300.0) / 1000.0
        self.s4_publisher.publish(s4_msg)

def main(args=None):
    rclpy.init(args=args)

    serial_node = SerialNode()

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
