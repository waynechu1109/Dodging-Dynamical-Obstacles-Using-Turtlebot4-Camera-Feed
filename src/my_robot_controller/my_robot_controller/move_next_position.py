def move_next_position(see_obstacle, current_obstacle_position, current_robot_position, obstacle_x_velo, obstacle_y_velo, robot_x_velo, robot_y_velo, robot_omega, sampling_freq, controller_index, robot_velocity):
    # global see_obstacle, current_obstacle_position, current_robot_position, obstacle_x_velo, obstacle_y_velo, robot_x_velo, robot_y_velo, robot_omega, sampling_freq, controller_index, robot_velocity
    current_obstacle_position[0] += obstacle_x_velo*(1/sampling_freq)
    current_obstacle_position[1] += obstacle_y_velo*(1/sampling_freq)
    current_robot_position[0] += robot_x_velo*(1/sampling_freq)
    current_robot_position[1] += robot_y_velo*(1/sampling_freq)
    current_robot_position[2] += robot_omega*(1/sampling_freq)
    # if not see_obstacle:
    #     current_robot_position[0] += robot_x_velo*(1/sampling_freq)
    #     current_robot_position[1] += robot_y_velo*(1/sampling_freq)
    #     current_robot_position[2] += robot_omega*(1/sampling_freq)
    # else:
    #     controller_index = 0
    #     robot_velocity = 0
    #     robot_omega = 0
