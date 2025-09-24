import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Point, PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray
from .dwa import DWA
from .utils import LaserScanSensor, range_to_obstacles, landmark_to_goal_pose
from rclpy.qos import qos_profile_sensor_data

class Controller(Node):
    """
    J = α ⋅ heading + β ⋅ vel + γ ⋅ dist_obs
    """ 

    def __init__(self):
        super().__init__(node_name='controller')

        self.debug = True
        self.task = 2

        # Dwa design parameters
        self.alpha = 0.03 if self.task == 1 else 0.9 if self.task == 2 else 0.4
        self.beta = 0.16 if self.task == 1 else 0.24 if self.task == 2 else 0.08
        self.gamma = 0.21 if self.task == 1 else 0.11 if self.task == 2 else 0.55
        self.sigma = 0.0 if self.task == 1 else 0.11 if self.task == 2 else 0.75

        # Filter design parameters
        self.num_ranges = 20 if self.task != 3 else 30 # [12 - 30]
        self.max_sensor_dist = 3.5 # [m]
        self.min_sensor_dist = 0.1 # [m]

        # Safety parameters
        self.collision_tol = 0.15 # [m]
        

        # dwa original implementantion already computes heading with x-y robot coordinates

        # Problem specifications
        control_freq = 15 # [Hz] 
        self.robot_radius = 0.1 # [m] # TODO: measure the real robot radius

        # parametros para realocar
        self.goal_dist_tol = 0.1


        distance_threshold_exp_vel = float('-inf') if self.task == 1 else 0.25 if self.task == 2 else 0.25
        keep_sight = self.task != 1
        kept_distance = 0.0 if self.task == 1 else 0.3 if self.task == 2 else 0.2

        init_pose = np.array([0.,0.,0.])

        # Create timers
        control_period = 1./control_freq # [s]
        self.timer_control = self.create_timer(control_period, self.control_callback)
        self.N = 0 # control steps
        self.feedback_rate = 50
        self.allowed_steps = 1000

        max_linear_velocity = 0.22 if self.task == 1 else 0.2 if self.task == 2 else 0.15
        max_angular_velocity = 1.5 if self.task == 1 else 1.5 if self.task == 2 else 1.5

        self.dwa_controller = DWA(
            dt = 0.07,
            sim_time = 2.0,
            v_samples = 10,
            w_samples = 20,
            goal_dist_tol = self.goal_dist_tol,
            collision_tol = self.collision_tol,
            weight_angle = self.alpha,
            weight_vel = self.beta,
            weight_obs = self.gamma,
            distance_threshold_exp_vel=distance_threshold_exp_vel,
            keep_sight=keep_sight,
            weight_sight=self.sigma,
            kept_distance=kept_distance,
            init_pose = init_pose,
            max_linear_acc = 0.5,
            max_ang_acc = 3.2,
            max_lin_vel = max_linear_velocity, # m/s
            min_lin_vel = 0.0, # m/s
            max_ang_vel = max_angular_velocity, # rad/s 
            min_ang_vel = -max_angular_velocity, # rad/s 
            radius = self.robot_radius, # m
        )

        # Subscribe to the laserscan topic to detect objects ahead
        #self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_callback, 10)
        if self.task == 3:
            self.subscription = self.create_subscription(LandmarkArray, '/camera/landmarks', self.landmark_callback, 10)

        # Create a publisher for command
        self.cmd_publisher_ = self.create_publisher(msg_type=Twist, topic='/cmd_vel', qos_profile=10)
        self.twist = Twist()

        # Create publisher for debugging
        if self.debug:
            self.obstacles_publisher_ = self.create_publisher(msg_type=Marker, topic='/obstacles', qos_profile=10)
            self.goal_publisher_ = self.create_publisher(msg_type=PointStamped, topic='/pose_goal', qos_profile=10)
            self.distance_goal_publisher_ = self.create_publisher(msg_type=Float32, topic='/distance_goal', qos_profile=10)
            self.goal_landmark_publisher_ = self.create_publisher(msg_type=PointStamped, topic='/goal_landmark', qos_profile=10)

        # initialize parameters
        self.laser = None
        self.laser_msg = None
        self.odom_msg = None
        self.goal_pose = None
        self.filtered_ranges = None

        # Safety control
        self.safe = True

        self.end = False
    
    def check_safety(self):
        min_safe_distance = self.robot_radius + self.collision_tol
        # filtered_ranges will already be different than None because of the controll_callback() structure
        # if self.debug:
        #     print(min(self.filtered_ranges[:,0]))
        if any(self.filtered_ranges[:,0] < min_safe_distance):
            return False
        return True
    
    def filtered_scan_ranges_to_obstacles(self):
            if self.laser is None: # laser_msg will already be different than None because of the controll_callback() structure
                self.laser = LaserScanSensor(
                    max_dist=self.max_sensor_dist,
                    min_dist=self.min_sensor_dist,
                    num_points=self.num_ranges,
                    tot_num_points=len(self.laser_msg.ranges)
                )
            self.filtered_ranges = self.laser.process_data(np.array(self.laser_msg.ranges))
            end_points = range_to_obstacles(self.filtered_ranges, self.dwa_controller.robot.pose)

            if self.debug:
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'points'
                marker.id = 0
                marker.type = Marker.POINTS 
                marker.action = Marker.ADD

                marker.scale.x = 0.1  
                marker.scale.y = 0.1

                marker.color.r = 0.0
                marker.color.g = 1.0  
                marker.color.b = 0.0 
                marker.color.a = 1.0 

                for i, j in end_points:
                    point = Point()
                    point.x, point.y, point.z = i, j, 0.0
                    marker.points.append(point)

                self.obstacles_publisher_.publish(marker)
        
            return end_points

    def scan_callback(self, laser_msg):
        self.laser_msg = laser_msg

    def goal_callback(self, pose_msg):
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        if self.task != 1 or pose_msg.header.stamp.sec > 9:
            self.goal_pose = np.array([x,y])

        if self.debug:
            point = PointStamped()
            point.header.frame_id = 'odom'
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x, point.point.y, point.point.z = x, y, 0.0

            self.goal_publisher_.publish(point)

    def odom_callback(self, odom_msg):
        self.odom_msg = odom_msg

    def landmark_callback(self, landmark_msg):
        if len(landmark_msg.landmarks) != 0:
            sum_range = 0
            sum_bearing = 0
            for landmark in landmark_msg.landmarks:
                sum_range += landmark.range
                sum_bearing += landmark.bearing
            landmark_data = np.array([sum_range/len(landmark_msg.landmarks), sum_bearing/len(landmark_msg.landmarks)])
            
            self.goal_pose = landmark_to_goal_pose(landmark_data, self.dwa_controller.robot.pose)

            if self.debug:
                point = PointStamped()
                point.header.frame_id = 'odom'
                point.header.stamp = self.get_clock().now().to_msg()
                point.point.x, point.point.y = self.goal_pose

                self.goal_landmark_publisher_.publish(point)

    def control_callback(self):
        sensor_data_is_available = self.laser_msg is not None
        goal_pose_is_set = self.goal_pose is not None
        if not goal_pose_is_set or not sensor_data_is_available or self.end:
            return

        # 1. Check if Goal reached
        dist_to_goal = np.linalg.norm(self.dwa_controller.robot.pose[0:2] - self.goal_pose)
        if dist_to_goal < self.goal_dist_tol:
            self.end = True
            print("Goal reached!")

        # 2. Get new observations for obstacles is needed
        # compute obstacles with lidar data
        obstacles = self.filtered_scan_ranges_to_obstacles()
        self.laser_msg = None # discard message already used

        # 3. Compute command for the robot with DWA controller
        safe = self.check_safety()
        if safe:
            u = self.dwa_controller.compute_cmd(self.goal_pose, self.dwa_controller.robot.pose, obstacles)
        else:
            u = np.array([0.,0.])
            self.end = True
            print("Collision! Robot is to close to obstacle.")
        
        # 4. Send the command or update the robot pose
        self.dwa_controller.robot.update_state(self.odom_msg, u)

        # 5. Provide intermediate task feedback
        if self.N % self.feedback_rate == 0:
            dist_to_goal = np.linalg.norm(self.dwa_controller.robot.pose[0:2] - self.goal_pose)
            float_msg = Float32()
            float_msg.data = dist_to_goal
            self.distance_goal_publisher_.publish(float_msg)

        self.N += 1

        if self.N > self.allowed_steps:
            self.end = True
            print("Timeout! Goal not reached.")

        # publish cmd_vel
        self.twist.linear.x = u[0]
        self.twist.angular.z = u[1]
        self.cmd_publisher_.publish(self.twist)
    
def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
