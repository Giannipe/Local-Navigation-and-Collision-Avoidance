# Obstacle Avoidance and Robot Following with Dynamic Window Approach

## 🎯 Objectives and Scope
The purpose of this project is to develop a **local navigation planner** capable of generating feasible trajectories that:

1. Respect the robot’s **kinematic constraints** (linear and angular velocity/acceleration limits).
2. **Avoid obstacles** that may appear after global planning.
3. **Track a moving goal** (in this case, another robot or a marker) in real-time.

To achieve this, the **Dynamic Window Approach (DWA)** was implemented within a ROS2 node. The system integrates sensor data, goal updates, and control laws to generate safe, collision-free trajectories that guide the robot toward the moving target.

## 🧩 Dynamic Window Approach – Core Idea
The DWA restricts the search space of possible velocities by considering three sets:
- ***Vs*** (velocity space): all velocities allowed by the robot’s physical limits.
- ***Vd*** (dynamic window): velocities reachable within the next control interval.
- ***Va*** (admissible velocities): velocities that are collision-free given current sensor data.

The intersection of these sets defines the feasible velocity window. Each candidate velocity pair (v, ω) is simulated, and the one maximizing the objective function is chosen:

$$
G(v, \omega) = \sigma \big( \alpha \cdot heading(v,\omega) + \beta \cdot vel(v,\omega) + \gamma \cdot dist(v,\omega) \big)
$$

Where:
- ***heading*** → alignment with the goal.
- ***vel*** → forward motion efficiency.
- ***dist*** → clearance from obstacles.
- ***σ*** → normalization.

This heuristic cost balances goal-seeking, speed, and safety.


## ⚙️ Task 1 – Static Goal Navigation

### Node Structure
The ROS2 node implements the DWA in a control loop running at ***15 Hz***, via a timer callback. Each cycle:
1. Checks if a goal is set and sensor data is available.
2. Updates the obstacle map from LiDAR.
3. Computes the control command using the DWA controller.
4. Applies safety checks and publishes the velocity command.
5. Provides debugging feedback (goal distance, obstacle visualization).

### Sensor Data Processing
LiDAR scan data may contain invalid readings (***NaN***, ***∞***). A preprocessing function:
- Replaces NaN with a minimum value, ∞ with max range.
- Saturates values to ***3.5 m***.
- Groups data by angular sectors, taking the minimum in each sector. This yields a compact and reliable obstacle set.
- 
```python
    def process_data(self, points):
        """Filter outlier values from ranges, subsample the desired num of ranges, saturate to max_dist"""

         # Clean scan data from nan and inf data
        cleaned_range = np.nan_to_num(points, nan=self.min_dist, posinf=self.max_dist)

        # Saturate ranges
        saturated_range = np.minimum(cleaned_range, self.max_dist)

        # Filter scan ranges and takes only num_points, choosing the minimum value
        # for each angular sector
        scan_range = []
        group_length = len(saturated_range)//self.num_points
        for i in range(self.num_points):
            grouped_range = saturated_range[i*(group_length):(i+1)*group_length]
            scan_range.append([min(grouped_range), np.argmin(grouped_range)+i*(group_length)])

        return np.array(scan_range)
```

### Obstacle Mapping
The obstacle coordinates are computed from LiDAR data and expressed in the **global frame** using linear transformations.  
Since the LiDAR is mounted on the robot’s front, each detected point in the **sensor frame** can be transformed into global coordinates through:

$$
X = A B p
$$

Where:  
- **A** → robot pose in the global frame  
- **B** → sensor pose in the robot frame  
- **p** → measured point in the sensor frame  

Given the robot pose $[x, y, \theta]$, the sensor offset $[a, b, \alpha]$ (constant during motion), and the LiDAR measurement $p = (d, 0)_{sensor}$, the obstacle position can be accurately mapped into the absolute frame.

$$ A = \left[
\begin{matrix}
\cos(\theta) & -\sin(\theta) & 0 & x \\
\sin(\theta) & \cos(\theta) & 0 & y \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{matrix}
\right], 
B = \left[
\begin{matrix}
\cos(\alpha) & -\sin(\alpha) & 0 & a \\
\sin(\alpha) & \cos(\alpha) & 0 & b \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{matrix}
\right],
p = \left[
\begin{matrix}
d \\ 0 \\ 0 \\ 1
\end{matrix}
\right]
$$

```python
 def range_to_obstacles(ranges, robot_pose):
        """""
        Compute the endpoint map coordinate from the scan range sensed
        """""
        robot_x, robot_y, robot_angle = robot_pose[:]
        a = -0.032 # simulation: -0.032
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

<p align="center">
  <img src="obstacles.png" alt="Obstacles in Rviz">
</p>
```

### Safety Mechanism
Using the set of filtered laser ranges, it was implemented a safety mechanism to stop the robot in cases of high proximity with obstacles. 

```python
    def check_safety(self):
        min_safe_distance = self.robot_radius + self.collision_tol
        if any(self.filtered_ranges[:,0] < min_safe_distance):
            return False
        return True
```

### Goal Management
Since the goal is moving, it is needed to update the target position from time to time. The goal manager assigns the new target position when it is recieved.

```python
    (...)
    self.subscription = self.create_subscription(Odometry, '/dynamic_goal_pose', self.goal_callback, 10)

    (...)

    def goal_callback(self, pose_msg):
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        self.goal_pose = np.array([x,y])
```

### Feedback & Debugging
To follow the DWA in action and help debugging, it was created a publisher to provide the current distance to the goal at every 50 times the *control_caballcak* function is called

```python
    (...)
	
    self.distance_goal_publisher_ = self.create_publisher(msg_type=Float32, topic='/distance_goal', qos_profile=10)

    (...)

        if self.N % self.feedback_rate == 0:
            dist_to_goal = np.linalg.norm(self.dwa_controller.robot.pose[0:2] - self.goal_pose)
            float_msg = Float32()
            float_msg.data = dist_to_goal
            self.distance_goal_publisher_(float_msg)
```

### 🚦 Control Callback – Main Loop
With all auxiliary functions in place, the **control_callback** function was completed with the full algorithmic structure.  
To manage task termination, print statements were added to indicate the event that ended the process:  
- **Goal** → the robot reached the target.  
- **Collision** → the safety mechanism was triggered.  
- **Timeout** → the robot failed to approach the target within the allowed time.

```python
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
```

## 🔎 Summary of Task 1
At this stage, the system successfully navigates toward a static goal, avoiding obstacles while respecting kinematic constraints. The core structure—sensor preprocessing, obstacle mapping, safety check, and DWA-based command generation—forms the foundation for later tasks involving dynamic goal tracking (Task 2) and real-world deployment (Task 3).


## 🏃 Task 2 – Dynamic Goal Following (Simulation)
In Task 2, the navigation system was extended to follow a **moving target** in simulation.  
The original DWA objective function was not sufficient, as the robot tended to move too fast near the target and could lose stability.  
For this reason, the cost function was modified with two main changes:

1. **Exponential velocity penalty** → slows the robot as it approaches the goal, ensuring smoother and more precise control.  
2. **Sightline maintenance term** → forces the robot to keep the moving target within its field of view, maintaining visibility and safe distance.  

The new objective function can be expressed as:

$$
G(v, \omega) = \sigma \big( \alpha \cdot heading(v, \omega) 
+ \beta \cdot vel(v, \omega) 
+ \gamma \cdot dist(v, \omega) 
+ \sigma \cdot sightline(v, \omega) \big)
$$

Where:  
- $heading(v, \omega)$ → alignment with the moving goal.  
- $vel(v, \omega)$ → velocity regulation with exponential decay near the target.  
- $dist(v, \omega)$ → obstacle clearance.  
- $sightline(v, \omega)$ → factor to keep the target visible.  

### 🔧 Tuned parameters
- $\alpha = 0.13$  
- $\beta = 0.04$  
- $\gamma = 0.32$  
- $\sigma = 0.37$  

### 📊 Results
- Success rate **above 90%** in Gazebo simulations.  
- Adaptive velocity: the robot slowed near the target and accelerated when the target moved further away.  
- Angular velocity corrections ensured that the target stayed within sight during turns.  
- Smooth trajectories with consistent obstacle avoidance.  

✅ **Outcome**: The modified DWA algorithm enabled robust and safe dynamic goal following in simulation.

# 🤖 Task 3 – Real Robot Experiments

In Task 3, the navigation system was deployed on a **real Turtlebot robot** equipped with LiDAR and an RGB-D camera for visual tracking.  
Unlike simulation, this phase introduced real-world challenges such as **sensor noise, latency, and varying lighting conditions**.  
The target was tracked using **AprilTags**, detected on the `/camera/landmarks` topic, and converted into goal positions for the controller.

### 🔄 Modifications to the algorithm
To ensure safe and stable operation, several adjustments were made:
1. **Visual tracking integration** → AprilTag detections were processed to estimate the target position, even when 0–3 tags were visible at a time.  
2. **Safety and filtering** → LiDAR data was filtered to remove noise and constrained to the 0.1–3.5 m range.  
3. **QoS profiles in ROS2** → subscriber reliability was improved to handle delays.  
4. **Parameter tuning** → the cost function was adapted for slower, safer movements.

The updated cost function emphasized keeping the target in sight and maintaining a safe following distance:

$$
G(v, \omega) = \sigma \big( \alpha \cdot heading(v, \omega) 
+ \beta \cdot vel(v, \omega) 
+ \gamma \cdot dist(v, \omega) 
+ \sigma \cdot sightline(v, \omega) \big)
$$

Where the weights now strongly prioritized **obstacle clearance** and **sightline maintenance** compared to Task 2.

### 🔧 Tuned parameters
- $\alpha = 0.40$  
- $\beta = 0.08$  
- $\gamma = 0.55$  
- $\sigma = 0.75$  
- Max linear velocity: **0.15–0.20 m/s**  
- Collision tolerance: **0.15 m**  

### 📊 Results
- Average distance error: **0.29 m RMSE**  
- Average bearing error: **0.18 rad RMSE**  
- Success rate: **≈ 90%** across different test scenarios (following another robot or a handheld AprilTag).  
- Failures occurred mainly when AprilTags were occluded, during sharp turns, or when speed limits were exceeded.  
- The robot always recovered tracking once the target was detected again.  

✅ **Outcome**: The system proved robust under real-world conditions, successfully combining LiDAR-based obstacle avoidance with visual target tracking.

# ✅ Conclusions
This project successfully designed, implemented, and tested a navigation pipeline enabling a robot to:

- Follow a moving target.
- Avoid static and dynamic obstacles.
- Operate both in simulation and on a real robot.

The combination of DWA trajectory planning, LiDAR-based obstacle mapping, and vision-based target detection proved effective and reliable. While some limitations remain in highly dynamic conditions, the work provides a solid foundation for extensions such as:

- Cooperative multi-robot tracking,
- Sensor fusion for localization,
- Adaptive/learning-based navigation strategies.
