import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
import math

class IMUtoOdometryNode(Node):
    def __init__(self):
        super().__init__('imu_to_odometry')
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            'unilidar/imu',  # IMU topic (adjust accordingly)
            self.imu_callback,
            10
        )
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            'unilidar/odom',
            10
        )

        self.last_time = self.get_clock().now()
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0] 
        
    def imu_callback(self, msg: Imu):

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  
        self.last_time = current_time
        
        accel = msg.linear_acceleration
        ang_vel = msg.angular_velocity
        
        self.velocity[0] += accel.x * dt
        self.velocity[1] += accel.y * dt
        self.velocity[2] += accel.z * dt
        
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt
                
        self.orientation[0] += ang_vel.x * dt
        self.orientation[1] += ang_vel.y * dt
        self.orientation[2] += ang_vel.z * dt
        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        odom.pose.pose.orientation = self.euler_to_quaternion(self.orientation[0],self.orientation[1],self.orientation[2])
        
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.linear.z = self.velocity[2]
        odom.twist.twist.angular.x = ang_vel.x
        odom.twist.twist.angular.y = ang_vel.y
        odom.twist.twist.angular.z = ang_vel.z
        
        self.odom_publisher.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        w = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        x = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        y = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        z = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        
        quat = Quaternion()
        quat.w = w
        quat.x = x 
        quat.y = y
        quat.z = z 
        return quat

def main(args=None):
    rclpy.init(args=args)
    node = IMUtoOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
