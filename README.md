# Obstacle Avoidance and Robot Following with Dynamic Window Approach
# 🎯 Objectives and Scope

The purpose of this project is to develop a local navigation planner capable of generating feasible trajectories that:

1. Respect the robot’s kinematic constraints (linear and angular velocity/acceleration limits).
2. Avoid obstacles that may appear after global planning.
3. Track a moving goal (in this case, another robot or a marker) in real-time.

To achieve this, the Dynamic Window Approach (DWA) was implemented within a ROS2 node. The system integrates sensor data, goal updates, and control laws to generate safe, collision-free trajectories that guide the robot toward the moving target.


# 🧩 Dynamic Window Approach – Core Idea

The DWA restricts the search space of possible velocities by considering three sets:

- Vs (velocity space): all velocities allowed by the robot’s physical limits.
- Vd (dynamic window): velocities reachable within the next control interval.
- Va (admissible velocities): velocities that are collision-free given current sensor data.

The intersection of these sets defines the feasible velocity window. Each candidate velocity pair (v, ω) is simulated, and the one maximizing the objective function is chosen:

$$
G(v, \omega) = \sigma \big( \alpha \cdot heading(v,\omega) + \beta \cdot vel(v,\omega) + \gamma \cdot dist(v,\omega) \big)
$$

Where:

- heading → alignment with the goal.
- vel → forward motion efficiency.
- dist → clearance from obstacles.
- σ → normalization.

This heuristic cost balances goal-seeking, speed, and safety.


# ⚙️ Task 1 – Static Goal Navigation
## Node Structure

The ROS2 node implements the DWA in a control loop running at 15 Hz, via a timer callback. Each cycle:

1. Checks if a goal is set and sensor data is available.
2. Updates the obstacle map from LiDAR.
3. Computes the control command using the DWA controller.
4. Applies safety checks and publishes the velocity command.
5. Provides debugging feedback (goal distance, obstacle visualization).

## Sensor Data Processing

LiDAR scan data may contain invalid readings (NaN, ∞). A preprocessing function:

- Replaces NaN with a minimum value, ∞ with max range.
- Saturates values to 3.5 m.
- Groups data by angular sectors, taking the minimum in each sector. This yields a compact and reliable obstacle set.

''' def process_data(self, points):
    cleaned = np.nan_to_num(points, nan=self.min_dist, posinf=self.max_dist)
    saturated = np.minimum(cleaned, self.max_dist)
    ...
    return np.array(scan_range) '''

## Obstacle Mapping

To convert LiDAR readings into global coordinates, homogeneous transformations are applied:

𝑋
=
𝐴
⋅
𝐵
⋅
𝑝
X=A⋅B⋅p

where:

A = robot pose in global frame,

B = sensor pose on robot,

p = obstacle in sensor frame.

def range_to_obstacles(ranges, robot_pose):
    ...
    coord = np.einsum('ij,njk,nk->ni', A, B, p)
    return coord[:,:2]

# Safety Mechanism

A check halts the robot if any obstacle is closer than a safety threshold:

def check_safety(self):
    min_safe_distance = self.robot_radius + self.collision_tol
    return not any(self.filtered_ranges[:,0] < min_safe_distance)

# Goal Management

The moving target’s pose is continuously updated through ROS2 subscriptions:

def goal_callback(self, pose_msg):
    self.goal_pose = np.array([pose_msg.pose.pose.position.x,
                               pose_msg.pose.pose.position.y])

# Feedback & Debugging

Intermediate feedback is published (distance to goal), aiding real-time monitoring in RViz.

# 🚦 Control Callback – Main Loop

The control loop integrates all components:

Goal check → if reached, stop.

Obstacle processing → update obstacle set.

Command computation → run DWA controller.

Safety check → stop if too close to obstacles.

Command publishing → update robot state.

Feedback → publish distance to goal.

Exit conditions → goal reached, collision detected, or timeout.

def control_callback(self):
    if not goal_pose_is_set or not sensor_data_is_available or self.end:
        return
    ...
    if dist_to_goal < self.goal_dist_tol:
        self.end = True
        print("Goal reached!")
    ...
    if not self.check_safety():
        u = np.array([0.,0.])
        self.end = True
        print("Collision!")

# 🔎 Summary of Task 1

At this stage, the system successfully navigates toward a static goal, avoiding obstacles while respecting kinematic constraints. The core structure—sensor preprocessing, obstacle mapping, safety check, and DWA-based command generation—forms the foundation for later tasks involving dynamic goal tracking (Task 2) and real-world deployment (Task 3).


# 🏃 Task 2 – Dynamic Goal Following (Simulation)

In Task 2, the DWA cost function was extended to handle moving targets.

# Modifications

Velocity regulation near goal:
An exponential penalty term was added to slow the robot down as it approaches the target.

𝑣
𝑒
𝑙
𝑝
𝑒
𝑛
𝑎
𝑙
𝑡
𝑦
=
𝑒
−
𝑑
𝑡
ℎ
𝑟
𝑒
𝑠
ℎ
𝑜
𝑙
𝑑
vel
penalty
	​

=e
−
threshold
d
	​


Sightline maintenance:
A new term weighted by σ = 0.37 ensures the robot keeps the moving target in view while maintaining safe distance.

Parameter tuning:

α = 0.13 (heading)

β = 0.04 (velocity)

γ = 0.32 (obstacle clearance)

σ = 0.37 (sightline factor)

# Results in Simulation

Robot successfully tracked moving targets in Gazebo with >90% success rate.

Smooth trajectories, adaptive speed, and safe obstacle avoidance.

Velocity profiles showed adaptive slowdown near the goal and acceleration when the target moved ahead.

Angular velocity adjustments aligned with sharp turns, keeping the target visible.

# Outcomes

Validated the DWA modifications for dynamic environments.

Demonstrated trade-offs between distance regulation and obstacle clearance.

Established a simulation framework to transition toward real-world tests.

# 🤖 Task 3 – Real Robot Experiments

Task 3 extended the navigation strategy to real-world testing with a Turtlebot.

## Implementation

- Visual tracking: AprilTags detected through RGB-D camera (/camera/landmarks).
- Dynamic goals: Either another robot or handheld markers.
- Control parameters (tuned for safety):
-- α = 0.4 (heading)
-- β = 0.08 (velocity)
-- γ = 0.55 (obstacle clearance)
-- σ = 0.75 (sightline factor)
- Hardware constraints:
-- Max linear velocity = 0.15–0.20 m/s
-- Sensor range = 0.1–3.5 m
-- Collision tolerance = 0.15 m

## Challenges Addressed

- Sensor noise → applied filtering to handle spurious LiDAR readings.
- Target detection → adapted AprilTag size, accounted for 0–3 tag visibility.
- Latency & lighting → adjusted QoS profile for ROS2 subscribers.

## Experimental Results

- Success rate around 90% in controlled lab tests.
- Robot maintained ~0.2 m average distance from target with RMSE ≈ 0.29 m.
- Temporary failures when:
-- Tags were occluded,
-- Target executed sharp turns,
-- Velocity exceeded safety limits.
-Despite these, the robot consistently resumed following once the target was reacquired.

## Outcomes

- Demonstrated robust real-world performance despite environmental variability.
- Validated system architecture for dynamic goal tracking with obstacle avoidance.
- Identified limitations for improvement:
-- Better sensor fusion (LiDAR + visual odometry).
-- Enhanced control to anticipate sharp turns.

Adaptive parameter tuning with ML approaches.

# ✅ Conclusions

This project successfully designed, implemented, and tested a navigation pipeline enabling a robot to:

- Follow a moving target.
- Avoid static and dynamic obstacles.
- Operate both in simulation and on a real robot.

The combination of DWA trajectory planning, LiDAR-based obstacle mapping, and vision-based target detection proved effective and reliable. While some limitations remain in highly dynamic conditions, the work provides a solid foundation for extensions such as:

- Cooperative multi-robot tracking,
- Sensor fusion for localization,
- Adaptive/learning-based navigation strategies.
