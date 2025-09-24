import numpy as np
import math
from plot_utils import plot_obstacles, plot_trajectories, plot_velocities
from dwa import DWA
import matplotlib.pyplot as plt

def main():

    init_pose = np.array([1.0, 2.0, 0.0]) # initial x, y, theta of the robot
    goal_pose = np.array([5.5, 7.0]) # Goal [x,y] coordinate
    obstacles = np.array([[6, 8], [2, 5], 
                          [8.5, 5], [3, 3], 
                          [5,6], [4.5, 1.5],
                          [7, 3], [3, 7.5],
                          [3.5, 7], [8.4, 8]]) 

    # initialize plot with goal, init pose and obstacles
    fig, ax = plt.subplots(figsize=(6, 6))
    obst_legend = plot_obstacles(obstacles, ax)
    goal_legend = ax.scatter(goal_pose[0], goal_pose[1], marker="o", c="r", s=80, label="goal")
    init_pose_legend = ax.scatter(init_pose[0], init_pose[1], marker="s", c="g", s=60, label="init pose")

    controller = DWA(
        dt = 0.1,
        sim_time = 2.0,
        time_granularity = 0.1,
        v_samples = 10,
        w_samples = 20,
        goal_dist_tol = 0.2,
        collision_tol = 0.2,
        weight_angle = 0.1,
        weight_vel = 0.2,
        weight_obs = 0.08,
        obstacles_map = obstacles,
        init_pose = init_pose,
        max_linear_acc = 0.5,
        max_ang_acc = 3.2,
        max_lin_vel = 0.5, # m/s
        min_lin_vel = 0.0, # m/s
        max_ang_vel = 3.0, # rad/s 
        min_ang_vel = -3.0, # rad/s 
        radius = 0.3, # m
    )

    done, robot_poses = controller.go_to_pose(goal_pose)
    
    (path_legend,) = ax.plot(robot_poses[:, 0], robot_poses[:, 1], "--", label="Robot path")
    ax.set_title("Nav to goal")
    ax.legend(handles=[goal_legend, init_pose_legend, obst_legend, path_legend])
    plt.show()

    plot_velocities(robot_poses[:, 3:])

if __name__ == "__main__":
    main()
