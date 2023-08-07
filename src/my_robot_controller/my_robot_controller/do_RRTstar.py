from RRT_star import RRTStar

def do_RRTstar(start, goal, obstacle_list, x_limit, y_limit, step_size, max_iterations, current_robot_position):
    # global start, goal, obstacle_list, x_limit, y_limit, step_size, max_iterations
    rrt_star = RRTStar(start, goal, obstacle_list, x_limit, y_limit, step_size, max_iterations)
    path = rrt_star.find_path()
    if path is not None:
        print("Path found!")
        print(path)
        print("The robot orientation: ", current_robot_position[2])
        # x_sp, y_sp = rrt_star.bspline_interpolation(path)  # Get the B-spline interpolated curve
        # plt.plot(x_sp, y_sp[1], color='purple')  # Plot the B-spline curve
        rrt_star.plot(path)
        return path
    else:
        print("Path not found.")
        return None
