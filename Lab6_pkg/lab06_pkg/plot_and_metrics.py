from rosbag2_reader_py import Rosbag2Reader
from rclpy.time import Time
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import tf_transformations
from utils import landmark_to_goal_pose, range_to_obstacles, LaserScanSensor


# -------------------------------------------------------------------------------------------------------------------------

'''
Parameters to modify: task, fov_deg, path and plot_type only
'''

task = 1
path = f"~/lab_ws/task{task}"

plot_type = 'xy' # 'x' or 'xy'
fov_deg = 60 # fov of the camera in simulation


# ------------------------------------------------------------------------------------------
simulation = 0
real_case = 1
mode = 1 * (task == 3)
kept_distance = 0.0 if task == 1 else 0.3 if task == 2 else 0.2
kept_bearing = 0.0
# --------------------------------------------------------------------------------------------------------------------------

if mode == real_case:
    sel_topics = ["/odom", "/scan", "/cmd_vel", '/camera/landmarks']
    delta_x = 0.
    delta_y = 0.
    delta_theta = 0.
elif mode == simulation:
    sel_topics = ["/odom", "/scan", "/cmd_vel", "/ground_truth", "/dynamic_goal_pose"]
    delta_x = 0.
    delta_y = 0.
    delta_theta = 0.

reader = Rosbag2Reader(path)
topics = reader.all_topics
print(topics)

tot_msgs = 0
for _ in reader:
    tot_msgs += 1

print(f"Total messages: {tot_msgs}")

tot_msgs = 0
reader.set_filter(sel_topics)
for _ in reader:
    tot_msgs += 1

print("After the filter is applyed: ", reader.selected_topics)
print(f"Total messages: {tot_msgs}")

for topic_name, msg, t in reader:
    print(f"Received message of type {type(msg).__name__} on topic {topic_name} recorded at time {t}")
    if type(msg) is Odometry:
        time = Time.from_msg(msg.header.stamp).nanoseconds
        print(f"Position (x, y) at time {time}: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
    break

time_gt = [] # ground truth
gt_data = []
time_odom = []
odom_data = []
scan_data = []
time_dg = [] # dynamic goal
dg_data = []
minimum_distance = float('inf')
max_dist = 3.5
goal_dist_tol = 0.15
tracking = []
time_goal = []
goal_distance_data = []
goal_bearing_data = []
vel_data = []
time_camera = []
camera_data = []

def get_yaw(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    _, _, yaw = tf_transformations.euler_from_quaternion(quat)
    return yaw

last_pose = np.array([0., 0., 0.])
initial_goal_pose = np.array([0., 0.])
goal_pose = initial_goal_pose
fov = fov_deg/180 * np.pi
stop_vel = (0., 0.)
last_cmd_vel = stop_vel

def bearing_to_goal(last_pose, goal_pose):
    angle_to_goal = np.arctan2(goal_pose[1] - last_pose[1], goal_pose[0] - last_pose[0])
    bearing_to_goal = angle_to_goal - last_pose[2]
    if abs(bearing_to_goal) > np.pi: bearing_to_goal -= np.sign(bearing_to_goal) * 2 * np.pi
    return bearing_to_goal

empty_times = 0

for topic_name, msg, t in reader:
    if topic_name == "/ground_truth":
        time_gt.append(Time.from_msg(msg.header.stamp).nanoseconds)
        gt_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y, get_yaw(msg)))
    elif topic_name == "/odom":
        if not np.array_equal(goal_pose, initial_goal_pose) and mode == simulation:
            distance_to_goal = np.linalg.norm(last_pose[0:2] - goal_pose)
            if distance_to_goal > 5: # case when the dynamic goal goes back to start in task 2
                break
            time_goal.append(Time.from_msg(msg.header.stamp).nanoseconds)
            goal_distance_data.append(distance_to_goal)
            bearing = bearing_to_goal(last_pose=last_pose, goal_pose=goal_pose)
            tracking.append(1 * (bearing < fov/2))
            goal_bearing_data.append(bearing)
        time_odom.append(Time.from_msg(msg.header.stamp).nanoseconds)
        odom_data.append((msg.pose.pose.position.x+delta_x, msg.pose.pose.position.y+delta_y, get_yaw(msg)+delta_theta))
        last_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, get_yaw(msg)])
    elif topic_name == "/scan":
        points = msg.ranges
        # Clean scan data from nan and inf data
        cleaned_range = [point for i, point in enumerate(points) if not np.isnan(point) and abs(points[i] - points[i-2]) < 0.3 and abs(points[(i+2)%len(points)] - points[i]) < 0.3]
        cleaned_range = np.nan_to_num(cleaned_range, posinf=max_dist)
        # Saturate ranges
        saturated_range = np.minimum(cleaned_range, max_dist)
        scan_data.append(np.mean(saturated_range))
        if np.min(saturated_range) < minimum_distance:
            minimum_distance = np.min(saturated_range)
            #print(saturated_range[np.argmin(saturated_range)-3:np.argmin(saturated_range)+3])
        
    elif topic_name == "/dynamic_goal_pose":
        goal_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        distance_to_goal = np.linalg.norm(last_pose[0:2] - goal_pose)
        if distance_to_goal < 5:
            time_dg.append(Time.from_msg(msg.header.stamp).nanoseconds)
            dg_data.append((msg.pose.pose.position.x, msg.pose.pose.position.y, get_yaw(msg)))
        if distance_to_goal < goal_dist_tol:
            break
    elif topic_name == "/cmd_vel":
        last_cmd_vel = (msg.linear.x, msg.angular.z)
        vel_data.append(last_cmd_vel)
    elif topic_name == "/camera/landmarks":
        time_camera.append(Time.from_msg(msg.header.stamp).nanoseconds)
        #print(msg.landmarks)
        if len(msg.landmarks) != 0:
            sum_range = 0
            sum_bearing = 0
            for landmark in msg.landmarks:
                sum_range += landmark.range
                sum_bearing += landmark.bearing
            landmark_data = np.array([sum_range/len(msg.landmarks), sum_bearing/len(msg.landmarks)])
            
            goal_pose = landmark_to_goal_pose(landmark_data, last_pose)
            time_goal.append(Time.from_msg(msg.header.stamp).nanoseconds)
            goal_distance_data.append(sum_range/len(msg.landmarks))
            goal_bearing_data.append(sum_bearing/len(msg.landmarks))
            empty_times = 0
        else:
            empty_times += 1
        norm = ((last_cmd_vel[0]-stop_vel[0])**2+(last_cmd_vel[1]-stop_vel[1])**2)**(1/2)
        if empty_times > 5 and norm < 0.02:
            tracking.append(0)
        else:
            tracking.append(1)
        if not np.array_equal(goal_pose, initial_goal_pose):
            camera_data.append(goal_pose + [delta_x, delta_y])  


average_distance = np.mean(scan_data)
print(f"Average distance to obstacles: {average_distance}")
print(f"Minimum distance to obstacles: {minimum_distance}")
if len(tracking) != 0:
    print(f"Time of tracking [%]: {100 * sum(tracking)/len(tracking)}")

time_gt = np.array(time_gt)
gt_data = np.array(gt_data)
time_odom = np.array(time_odom)
odom_data = np.array(odom_data)


print(f"Ground truth points: {len(gt_data)}")
print(f"Odometry points: {len(odom_data)}")


if len(gt_data) > 0:
    gt_total_time = (time_gt[-1] - time_gt[0])/(10**(9))
    print(f"Time ground truth seconds: {gt_total_time}")
    num_points_gt = len(gt_data)

odom_total_time = (time_odom[-1] - time_odom[0])/(10**(9))
print(f"Time odom seconds: {odom_total_time}")
num_points_odom = len(odom_data)

if len(dg_data) > 0:
    dg_total_time = (time_dg[-1] - time_dg[0])/(10**(9))
    print(f"Time dg seconds: {dg_total_time}")
    num_points_dg = len(dg_data)

if len(goal_distance_data) > 0:
    goal_total_time = (time_goal[-1] - time_goal[0])/(10**(9))
    print(f"Time goal seconds: {goal_total_time}")
    num_points_goal = len(goal_distance_data)

if len(camera_data) > 0:
    camera_total_time = (time_camera[-1] - time_camera[0])/(10**(9))
    print(f"Time camera seconds: {camera_total_time}")
    num_points_camera = len(camera_data)


plot_dict = {'x': (0,0), 'y': (0,1), 'theta': (0,2), 'xy': (1,1)}

# State plots

title_list = ['Robot horizontal position in time for different topics', 'Robot vertical position in time for different topics',
              'Robot orientation in time for different topics', 'Robot trajectory on XY plane for different topics']
xlabel_list = ['Time (s)', 'X (m)']
ylabel_list = ['X (m)', 'Y (m)', 'θ (rad)']

state_plots = ['x', 'y', 'theta']

def plot_curve(axis1, axis2, ax=None):
    title = title_list[axis1+axis2]
    xlabel = xlabel_list[axis1]
    ylabel = ylabel_list[axis2]
    # Plot Odometry
    axis1_odom_list = [np.linspace(0, odom_total_time, num_points_odom), [data[0] for data in odom_data]]
    axis1_odom = axis1_odom_list[axis1]
    axis2_odom = [data[axis2] for data in odom_data]
    if ax is not None:
        ax.plot(
            axis1_odom, 
            axis2_odom, 
            '-o', 
            label="odom",
            markersize=1
        )
    else:
        plt.plot(
            axis1_odom, 
            axis2_odom, 
            '-o', 
            label="odom",
            markersize=1
        )
    # Plot Ground truth
    if len(gt_data) > 0:
        axis1_gt_list = [np.linspace(0, gt_total_time, num_points_gt), [data[0] for data in gt_data]]
        axis1_gt = axis1_gt_list[axis1]
        axis2_gt = [data[axis2] for data in gt_data]
        if ax is not None:
            ax.plot(
                axis1_gt, 
                axis2_gt, 
                '-o', 
                label="ground_truth",
                markersize=1
            )
        else:
            plt.plot(
                axis1_gt, 
                axis2_gt, 
                '-o', 
                label="ground_truth",
                markersize=1
            )
    if len(dg_data) > 0:
        # Plot Dynamic goal
        axis1_dg_list = [np.linspace(0, dg_total_time, num_points_dg), [data[0] for data in dg_data]]
        axis1_dg = axis1_dg_list[axis1]
        axis2_dg = [data[axis2] for data in dg_data]
        if ax is not None:
            ax.plot(
                axis1_dg, 
                axis2_dg, 
                '-o', 
                label="target",
                markersize=1
            )
        else:
            plt.plot(
                axis1_dg, 
                axis2_dg, 
                '-o', 
                label="target",
                markersize=1
            )
    if len(camera_data) > 0:
        # Plot target estimation pose by camera data
        axis1_camera_list = [np.linspace(0, camera_total_time, num_points_camera), [data[0] for data in camera_data]]
        axis1_camera = axis1_camera_list[axis1]
        axis2_camera = [data[axis2] for data in camera_data]
        if ax is not None:
            ax.plot(
                axis1_camera, 
                axis2_camera, 
                '-o', 
                label="target",
                markersize=1
            )
        else:
            plt.plot(
                axis1_camera, 
                axis2_camera, 
                '-o', 
                label="target",
                markersize=1
            )

    if ax is not None:
        ax.legend(loc="upper right")
        #ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid()
    else:
        plt.legend(loc="upper right")
        #plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.grid()

    
axis1, axis2 = plot_dict[plot_type]

if plot_type in state_plots:
    fig, ax = plt.subplots(3, 1, figsize=(12, 8))
    for state in state_plots:
        axis1, axis2 = plot_dict[state]
        plot_curve(axis1, axis2, ax=ax[axis2])
    fig.suptitle('Robot pose parameters in time for different topics')
    plt.tight_layout()
    plt.show()
else:
    plt.figure(figsize=(12,10))
    plot_curve(axis1, axis2)
    plt.title(title_list[-1])
    plt.show()


if len(goal_distance_data) > 0:
    desired_distance = kept_distance
    desired_bearing = kept_bearing
    goal_distance_data = np.array(goal_distance_data)
    goal_bearing_data = np.array(goal_bearing_data)

    dist_rmse = np.sqrt(np.mean((goal_distance_data - desired_distance) ** 2))
    bearing_rmse = np.sqrt(np.mean((goal_bearing_data - desired_bearing) ** 2))

    print('\n\n')
    print('Computed metrics between distance and bearing to goal and optimal values')
    print(f'distance RMSE: {dist_rmse}')
    print(f'bearing RMSE: {bearing_rmse}')

    plt.figure()
    plt.plot(
        np.linspace(0, goal_total_time, num_points_goal), 
        goal_distance_data - desired_distance, 
        label="Error",
        markersize=1
    )


    plt.axhline(dist_rmse, color='k', linestyle='--', label=f"RMSE = {dist_rmse:.5f}")
    #plt.axhline(mae_, color='r', linestyle='--', label=f"MAE = {mae_:.5f}")
    plt.title("Error with respect to optimal distance between robot and target")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.legend()
    plt.grid()

    plt.show()

    plt.figure()
    plt.plot(
        np.linspace(0, goal_total_time, num_points_goal), 
        goal_bearing_data - desired_bearing, 
        label="Error",
        markersize=1
    )


    plt.axhline(bearing_rmse, color='k', linestyle='--', label=f"RMSE = {bearing_rmse:.5f}")
    #plt.axhline(mae_, color='r', linestyle='--', label=f"MAE = {mae_:.5f}")
    plt.title("Error with respect to optimal bearing between robot and target")
    plt.xlabel("Time (s)")
    plt.ylabel("Error (rad)")
    plt.legend()
    plt.grid()

    plt.show()

if len(vel_data) > 0:
    vel_data = np.array(vel_data)
    # Sample data
    time = np.linspace(0, odom_total_time, len(vel_data))

    # Dual-axis plot
    fig, ax1 = plt.subplots()

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Linear Velocity (v) (m/s)', color='tab:blue')
    ax1.plot(time, vel_data[:,0], color='tab:blue')
    ax1.tick_params(axis='y', labelcolor='tab:blue')

    ax2 = ax1.twinx()
    ax2.set_ylabel('Angular Velocity (ω) (rad/s)', color='tab:red')
    ax2.plot(time, vel_data[:,1], color='tab:red')
    ax2.tick_params(axis='y', labelcolor='tab:red')

    plt.title('Velocities Profile (v, ω)')
    plt.show()