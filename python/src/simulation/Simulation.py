import numpy as np
import simulation.generate_path
import simulation.create_simulations.save_simulation_MC



if __name__ == '__main__':
    # simulation settings ----------------------------------------------------------------------------------------------
    sensor_pos = np.array([[-5000, 0, 0],
                           [400, -7400, 0],
                           [800, 800, 0]])
    color_list = np.array(['r', 'g', 'y', 'k', 'm'])
    target_pos = np.array([0, 0, 5000])
    target_speed_xy = 50
    target_speed_z = 10
    target_rot_speed = 3
    time_res = 0.5
    should_generate_data = False
    animation = False
    # ------------------------------------------------------------------------------------------------------------------



    path1 = generate_path("initial_phi", 0, "initial_speed_xy", target_speed_xy, "initial_speed_z", target_speed_z,
                          "initial_position", target_pos, "TimeRes", time_res)
    path1.add_straight_interval(100)
    path1.add_xy_turn_interval(90, -deg2rad(target_rot_speed))
    path1.add_straight_interval(100)
    save_simulation_MC([[-5000, 0, 0],
                        [400, -7400, 0],
                        [800, 800, 0]], path1.path, path1.time, 100, "../data/non_iter_navidi_3_sensors_100.mat")

    ax1 = plot3(true_path(1:1, 1), true_path(1: 1, 2), true_path(1: 1, 3), 'b.-');
    for i = 1:length(true_path)
    delete(ax1);

    ax1 = plot3(true_path(1:i, 1), true_path(1: i, 2), true_path(1: i, 3), 'b.-');

    prediction_position_iter = estimated_path_iter(i,:);
    prediction_position_non_iter = estimated_path_non_iter(i,:);
    ax3 = plot3(prediction_position_iter(1), prediction_position_iter(2), prediction_position_iter(3), 'r*');
    ax3 = plot3(prediction_position_non_iter(1), prediction_position_non_iter(2), prediction_position_non_iter(3),
                'g*');
    pause(sleep_duration);
    delete(ax3);
    ax3 = plot3(prediction_position_iter(1), prediction_position_iter(2), prediction_position_iter(3), 'r.');
    ax3 = plot3(prediction_position_non_iter(1), prediction_position_non_iter(2), prediction_position_non_iter(3),
                'g.');
end