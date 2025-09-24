import numpy as np
import math
import tf_transformations


class Differential_drive_robot():
    def __init__(self, 
                init_pose,
                max_linear_acc = 0.8,
                max_ang_acc = 100 * math.pi /180,
                max_lin_vel = 1.0, # m/s
                min_lin_vel = 0.0, # m/s
                max_ang_vel = 3.0, # rad/s 
                min_ang_vel = -3.0, # rad/s 
                radius = 0.3, # radius for circular robot
                ):
        
        #initialization
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])
        
        # kinematic properties
        self.max_linear_acc = max_linear_acc
        self.max_ang_acc = max_ang_acc
        self.max_lin_vel = max_lin_vel
        self.min_lin_vel = min_lin_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel

        # size
        self.radius = radius # circular shape

        # trajectory initialization
        self.trajectory = np.array([init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0]).reshape(1, -1)

    def update_state(self, u, dt):
        """
        Compute next pose of the robot according to differential drive kinematics rule (platform level equation).
        Save velocity and pose in the overall trajectory list of configurations.
        """

        if u is list:
            u = np.array(u)

        self.vel = u

        next_x = self.pose[0] + self.vel[0] * math.cos(self.pose[2]) * dt
        next_y = self.pose[1] + self.vel[0] * math.sin(self.pose[2]) * dt
        next_th = self.pose[2] + self.vel[1] * dt
        self.pose = np.array([next_x, next_y, next_th])

        traj_state = np.array([next_x, next_y, next_th, self.vel[0], self.vel[1]]).reshape(1, -1)
        self.trajectory = np.concatenate([self.trajectory, traj_state], axis=0)

        return self.pose

class Robot():
    def __init__(self, 
                init_pose,
                max_linear_acc = 0.8,
                max_ang_acc = 100 * math.pi /180,
                max_lin_vel = 1.0, # m/s
                min_lin_vel = 0.0, # m/s
                max_ang_vel = 3.0, # rad/s 
                min_ang_vel = -3.0, # rad/s 
                radius = 0.3, # radius for circular robot
                ):
        
        #initialization
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])
        
        # kinematic properties
        self.max_linear_acc = max_linear_acc
        self.max_ang_acc = max_ang_acc
        self.max_lin_vel = max_lin_vel
        self.min_lin_vel = min_lin_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel

        # size
        self.radius = radius # circular shape

        # trajectory initialization
        self.trajectory = np.array([init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0]).reshape(1, -1)

    def update_state(self, odom_msg, u):
        """
        Convert the pose from odom_msg format to the used format [x,y,θ].
        Save velocity and pose in the overall trajectory list of configurations.
        """

        if u is list:
            u = np.array(u)

        self.vel = u

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        quaternion = odom_msg.pose.pose.orientation
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, theta = tf_transformations.euler_from_quaternion(quat)
        self.pose = np.array([x,y,theta])

        traj_state = np.array([x, y, theta, self.vel[0], self.vel[1]]).reshape(1, -1)
        self.trajectory = np.concatenate([self.trajectory, traj_state], axis=0)


def calc_nearest_obs(robot_pose, obstacles, obstacle_max_dist=3):
    """
    Filter obstacles: find the ones in the local map considered for obstacle avoidance.
    """
    nearest_obs = []
    
    for obs in obstacles:
        temp_dist_to_obs = np.linalg.norm(robot_pose[0:2]-obs)

        if temp_dist_to_obs < obstacle_max_dist :
            nearest_obs.append(obs)

    return np.array(nearest_obs)

class LaserScanSensor:
    def __init__(
        self,
        max_dist,
        min_dist,
        num_points,
        tot_num_points,
    ):
        self.max_dist = max_dist
        self.min_dist = min_dist
        self.tot_num_points = tot_num_points
        self.num_points = num_points

    def process_data(self, points):
        """Filter outlier values from ranges, subsample the desired num of ranges, saturate to max_dist"""

         # Clean scan data from nan and inf data
        cleaned_range = np.nan_to_num(points, nan=self.max_dist, posinf=self.max_dist)

        # Saturate ranges
        saturated_range = np.minimum(cleaned_range, self.max_dist)

        # Filter scan ranges and takes only num_points, choosing the minimum value
        # for each angular sector
        scan_range = []
        group_length = len(saturated_range)//self.num_points
        for i in range(self.num_points):
            grouped_range = saturated_range[i*(group_length):(i+1)*group_length]
            #scan_range.append([np.mean(grouped_range), i*(group_length)+group_length//2]) # real case
            scan_range.append([min(grouped_range), np.argmin(grouped_range)+i*(group_length)]) # simulation
            
        return np.array(scan_range)

def range_to_obstacles(ranges, robot_pose):
    """""
    Compute the endpoint map coordinate from the scan range sensed
    """""
    robot_x, robot_y, robot_angle = robot_pose[:]
    a = -0.003 # simulation: -0.032
    b = 0.0 # simulation: 0.0
    angles = np.radians(ranges[:, 1])
    distances = ranges[:, 0] 

    # obstacles coordinates = Abp
    # A: express robot frame in absolute frame
    # B: express sensor frame in robot frame
    # p: coordinate of detected obstacles in sensor frame 
    A = np.array([
        [np.cos(robot_angle), -np.sin(robot_angle), 0, robot_x],
        [np.sin(robot_angle),  np.cos(robot_angle), 0, robot_y],
        [0,              0,       1, 0],
        [0,              0,       0, 1]
    ])
    B = np.array([[
        [np.cos(alpha), -np.sin(alpha), 0, a],
        [np.sin(alpha),  np.cos(alpha), 0, b],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ] for alpha in angles]) 
    p = np.array([[d, 0, 0, 1] for d in distances])
    
    coord = np.einsum('ij,njk,nk->ni', A, B, p)
    end_points = coord[:,:2]

    return end_points

def normalize_angle(theta):
    """
    Normalize angles between [-pi, pi)
    """
    theta = theta % (2 * np.pi)  # force in range [0, 2 pi)
    if np.isscalar(theta):
        if theta > np.pi:  # move to [-pi, pi)
            theta -= 2 * np.pi
    else:
        theta_ = theta.copy()
        theta_[theta>np.pi] -= 2 * np.pi
        return theta_
    
    return theta

def normalize(arr: np.ndarray):
    """ normalize array of values """
    if np.isclose(np.max(arr) - np.min(arr), 0.0):
        return np.zeros_like(arr)
    else:
        return (arr - np.min(arr)) / (np.max(arr) - np.min(arr))

def sigmoid(x: np.ndarray):
  """ compute sigmoid smoothing activation of a given array of values"""
  return 1/(1 + np.exp(-x)) 

def landmark_to_goal_pose(landmark, robot_pose):
    """ Recieves a landmark in the form [range, bearing] and robot pose [x,y,θ] """

    robot_x, robot_y, robot_angle = robot_pose[:]
    a = -0.047 # camera x coordinate in robot frame
    b = 0.0 # camera y coordinate in robot frame
    alpha = landmark[1]
    d = landmark[0] 

    # obstacles coordinates = Abp
    # A: express robot frame in absolute frame
    # B: express camera frame in robot frame
    # p: coordinate of detected obstacles in camera frame 
    A = np.array([
        [np.cos(robot_angle), -np.sin(robot_angle), 0, robot_x],
        [np.sin(robot_angle),  np.cos(robot_angle), 0, robot_y],
        [0,              0,       1, 0],
        [0,              0,       0, 1]
    ])
    B = np.array([
        [np.cos(alpha), -np.sin(alpha), 0, a],
        [np.sin(alpha),  np.cos(alpha), 0, b],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ]) 
    p = np.array([d, 0, 0, 1])
    
    coord = A@B@p.T
    goal_pose = coord[:2]

    return goal_pose

