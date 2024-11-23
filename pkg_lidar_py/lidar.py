import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Header
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster
import serial
import time
from sensor_msgs_py import point_cloud2 as pc2

class Lidar(Node):
    def __init__(self):
        super().__init__('lidarnode')
        self.distance_pub = self.create_publisher(Int32, 'distance', 10)
        self.strength_pub = self.create_publisher(Int32, 'strength', 10)
        self.temperature_pub = self.create_publisher(Float32, 'temperature', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.ser = serial.Serial("/dev/ttyS0", 115200)
        time.sleep(2)
        self.laserscan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        self.timer = self.create_timer(0.1, self.read_data)
        #self.timer = self.create_timer(0.1, self.publish_laserscan_and_pointcloud)

    def publish_laserscan_and_pointcloud(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_link"
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [5.0, 3.0, 1.5, 4.2, 3.3]
        self.laserscan_pub.publish(scan)
        self.get_logger().info(f"Published LaserScan data with {len(scan.ranges)} points")

        points = [(x,0.0,0.0) for x in scan.ranges]
        if not points:
            self.get_logger().warn("No points in the cloud")
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "laser_link"
        pc_data = pc2.create_cloud_xyz32(header, points)
        self.pointcloud_pub.publish(pc_data)
        self.get_logger().info(f"Published PointCloud2 with {len(points)} points")
        # pc = PointCloud2()
        # pc.header.stamp = self.get_clock().now().to_msg()
        # pc.header.frame_id = "laser_link"
        # pc.height = 1
        # pc.width = len(points)
        # pc.is_dense = True
        # pc.is_bigendian = False
        # pc.fields = [
        #     PointField(name="x", offset=0, datatype=PointField.INT32, count=1),
        #     PointField(name="y", offset=4, datatype=PointField.INT32, count=1),
        #     PointField(name="z", offset=8, datatype=PointField.INT32, count=1),
        # ]
        # pc.data = pc2.create_cloud_xyz32(pc.header, points)
        # self.pointcloud_pub.publish(pc)


    def read_data(self):
        if self.ser.in_waiting > 8:
            bytes_serial = self.ser.read(9)
            self.ser.reset_input_buffer()
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance = bytes_serial[2] + bytes_serial[3]*256
                strength = bytes_serial[4] + bytes_serial[5]*256
                temp = bytes_serial[6] + bytes_serial[7]*256
                temp = (temp/8) - 256

                self.get_logger().info(f"Distance: {distance}, Strength: {strength}, Temp: {temp}")

                self.distance_pub.publish(Int32(data=distance))
                self.strength_pub.publish(Int32(data=strength))
                if temp != 0:
                    self.temperature_pub.publish(Float32(data=temp))
                self.broadcast_transform()

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser_frame"

        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Transform broadcasted: base_link -> laser_frame")


    def destroy_node(self):
        if self.ser.is_open:
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
