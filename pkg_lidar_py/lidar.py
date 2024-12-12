import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String, Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import TransformBroadcaster
import serial
import time
#from sensor_msgs_py import point_cloud2 as pc2


class Lidar(Node):
    def __init__(self):
        super().__init__('lidarnode')

        # Declare parameters for serial port configuration
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Initialize serial communication
        try:
            self.ser = serial.Serial(serial_port, baud_rate)
            time.sleep(2)
            self.get_logger().info(f"Connected to Lidar on {serial_port} at {baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Lidar: {e}")
            self.ser = None

        # Publishers
        self.distance_pub = self.create_publisher(Int32, 'distance', 10)
        self.strength_pub = self.create_publisher(Int32, 'strength', 10)
        self.temperature_pub = self.create_publisher(Float32, 'temperature', 10)
        self.diagnostics_pub = self.create_publisher(String, 'lidar_diagnostics', 10)
        # self.laserscan_pub = self.create_publisher(LaserScan, 'scan', 10)
        # self.pointcloud_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        # self.diagnostics_pub = self.create_publisher(String, 'lidar_diagnostics', 10)

        # Transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for data reading
        self.timer = self.create_timer(0.1, self.read_data)

    def read_data(self):
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 8:
                    bytes_serial = self.ser.read(9)
                    print(bytes_serial)
                    self.ser.reset_input_buffer()
                    if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                        distance = bytes_serial[2] + bytes_serial[3] * 256
                        strength = bytes_serial[4] + bytes_serial[5] * 256
                        temp = bytes_serial[6] + bytes_serial[7] * 256
                        temp = (temp / 8) - 256

                        if distance > 0:
                            self.get_logger().info(f"Distance: {distance}, Strength: {strength}, Temp: {temp}")
                            self.distance_pub.publish(Int32(data=distance))
                            self.strength_pub.publish(Int32(data=strength))
                            if temp != 0:
                                self.temperature_pub.publish(Float32(data=temp))
                            # self.broadcast_transform()
                            # self.publish_laserscan_and_pointcloud(distance)
                        else:
                            self.get_logger().warn("Invalid distance data")
                            self.diagnostics_pub.publish(String(data="Invalid distance data"))
                    else:
                        self.get_logger().error("Unexpected data format from Lidar")
                        self.diagnostics_pub.publish(String(data="Unexpected data format from Lidar"))
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {e}")
                self.diagnostics_pub.publish(String(data=f"Serial communication error: {e}"))
        else:
            # Simulation mode for testing
            distance = 150  # Simulate 1.5 meters
            strength = 5000
            temp = 25.0
            self.get_logger().info(f"[Simulation] Distance: {distance}, Strength: {strength}, Temp: {temp}")
            self.distance_pub.publish(Int32(data=distance))
            self.strength_pub.publish(Int32(data=strength))
            self.temperature_pub.publish(Float32(data=temp))
            # self.broadcast_transform()
            # self.publish_laserscan_and_pointcloud(distance)

    # def publish_laserscan_and_pointcloud(self, distance):
    #     # LaserScan message
    #     scan = LaserScan()
    #     scan.header.stamp = self.get_clock().now().to_msg()
    #     scan.header.frame_id = "laser_link"
    #     scan.angle_min = -1.57
    #     scan.angle_max = 1.57
    #     scan.angle_increment = 0.01
    #     scan.range_min = 0.1
    #     scan.range_max = 12.0
    #     scan.ranges = [distance] * 315  # Simulate 315 points with the same distance
    #     self.laserscan_pub.publish(scan)
    #     self.get_logger().info(f"Published LaserScan with {len(scan.ranges)} points")

        # PointCloud2 message
        # points = [(distance * i * 0.01, 0.0, 0.0) for i in range(len(scan.ranges))]
        # header = Header()
        # header.stamp = self.get_clock().now().to_msg()
        # header.frame_id = "laser_link"
        # pc_data = PointCloud2.create_cloud_xyz32(header, points)
        # self.pointcloud_pub.publish(pc_data)
        # self.get_logger().info(f"Published PointCloud2 with {len(points)} points")

    # def broadcast_transform(self):
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = "base_link"
    #     t.child_frame_id = "laser_frame"

    #     t.transform.translation.x = 0.1
    #     t.transform.translation.y = 0.0
    #     t.transform.translation.z = 0.2
    #     t.transform.rotation.x = 0.0
    #     t.transform.rotation.y = 0.0
    #     t.transform.rotation.z = 0.0
    #     t.transform.rotation.w = 1.0

    #     self.tf_broadcaster.sendTransform(t)
    #     self.get_logger().info("Transform broadcasted: base_link -> laser_frame")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Lidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
