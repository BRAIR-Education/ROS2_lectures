# Import ROS Communication Library
import rclpy
from rclpy.node import Node

# Import Messages
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu

# Import math for calculations and noise generation
import numpy as np

# tf
from tf_transformations import quaternion_from_euler

QUEUE_SIZE = 10
GRAVITY_ACCELERATION = 9.80665 # [m/s^2]

class ImuSimulator(Node):

    def __init__(self):
        super().__init__('imu_simulator')

        # Subscriber and Publisher
        self.subscriber_ = self.create_subscription(
            Pose2D, 
            '/robot_state', 
            self.pose_callback, 
            QUEUE_SIZE
        )
        self.publisher_ = self.create_publisher(Imu, '/imu', QUEUE_SIZE)

        # Avoid unused variable warning
        self.subscriber_

        ## Sensor Noise Parameters (Standard Deviations)
        self.sigma_theta = 0.01  # [rad]
        self.sigma_omega = 0.05  # [rad/s]
        self.sigma_accel = 0.1   # [m/s^2]

        ## Class Elements for numerical differentiation
        self.prev_time = None
        
        # Previous Pose
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        
        # Previous Global Velocities
        self.prev_vx = 0.0
        self.prev_vy = 0.0

    def pose_callback(self, msg):
        """
        Callback triggered every time a new Pose2D is published.
        Computes the first and second derivatives to estimate IMU data.
        """
        current_time = self.get_clock().now()

        # Initialize previous state on the very first callback
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_x = msg.x
            self.prev_y = msg.y
            self.prev_theta = msg.theta
            return

        # Calculate delta time (dt) in seconds
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        # Prevent division by zero if messages arrive instantly
        if dt <= 0.0:
            return

        ### 1. Compute Ideal Global Velocities (First Derivative)
        vx = (msg.x - self.prev_x) / dt
        vy = (msg.y - self.prev_y) / dt

        # Compute angular velocity (wrap the angle difference to avoid jumps at pi/-pi)
        d_theta = msg.theta - self.prev_theta
        d_theta = np.arctan2(np.sin(d_theta), np.cos(d_theta))
        omega_ideal = d_theta / dt

        ### 2. Compute Ideal Global Accelerations (Second Derivative)
        ax = (vx - self.prev_vx) / dt
        ay = (vy - self.prev_vy) / dt

        ### 3. Transform Accelerations to the Body Frame
        accel_body_x_ideal = ax * np.cos(msg.theta) + ay * np.sin(msg.theta)
        accel_body_y_ideal = -ax * np.sin(msg.theta) + ay * np.cos(msg.theta)

        ### 4. Inject Gaussian Noise
        # N(mean=ideal_value, std_dev=sigma)
        theta_noisy = msg.theta + np.random.normal(0.0, self.sigma_theta)
        omega_noisy = omega_ideal + np.random.normal(0.0, self.sigma_omega)
        accel_x_noisy = accel_body_x_ideal + np.random.normal(0.0, self.sigma_accel)
        accel_y_noisy = accel_body_y_ideal + np.random.normal(0.0, self.sigma_accel)

        ### 5. Build and Publish the IMU Message
        self.publish_imu(theta_noisy, omega_noisy, accel_x_noisy, accel_y_noisy, current_time)

        ### 6. Update state variables for the next iteration 
        self.prev_time = current_time
        self.prev_x = msg.x
        self.prev_y = msg.y
        self.prev_theta = msg.theta
        self.prev_vx = vx
        self.prev_vy = vy

    def publish_imu(self, theta, omega, accel_x, accel_y, current_time):
        
        imu_msg = Imu()
        
        # Header setup
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = 'base_link'

        ### ORIENTATION
        # Convert Roll (0), Pitch (0), and Yaw (theta) to a Quaternion
        q = quaternion_from_euler(0.0, 0.0, theta)
        
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # Populate Orientation Covariance matrix (diagonal only)
        imu_msg.orientation_covariance[8] = self.sigma_theta ** 2

        ### ANGULAR VELOCITY
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = float(omega)

        # Populate Angular Velocity Covariance matrix
        imu_msg.angular_velocity_covariance[8] = self.sigma_omega ** 2

        ### LINEAR ACCELERATION
        imu_msg.linear_acceleration.x = float(accel_x)
        imu_msg.linear_acceleration.y = float(accel_y)
        # Add gravity to the Z axis, plus the sensor noise
        imu_msg.linear_acceleration.z = GRAVITY_ACCELERATION + np.random.normal(0.0, self.sigma_accel) 

        # Populate Linear Acceleration Covariance matrix
        imu_msg.linear_acceleration_covariance[0] = self.sigma_accel ** 2  # x-axis
        imu_msg.linear_acceleration_covariance[4] = self.sigma_accel ** 2  # y-axis
        imu_msg.linear_acceleration_covariance[8] = self.sigma_accel ** 2  # z-axis

        self.publisher_.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    imu_simulator = ImuSimulator()
    
    try:
        rclpy.spin(imu_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        imu_simulator.destroy_node()
        # Only shutdown if context is still valid
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()