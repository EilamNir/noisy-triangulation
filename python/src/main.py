import numpy as np
from simulation.generate_path import generate_path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import simulation.create_simulations.save_simulation_MC



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



    path1 = generate_path(0, target_speed_xy, target_speed_z, target_pos, time_res)
    path1.add_straight_interval(100)
    path1.add_xy_turn_interval(90, -np.deg2rad(target_rot_speed))
    path1.add_straight_interval(100)
    # save_simulation_MC([[-5000, 0, 0],
    #                     [400, -7400, 0],
    #                     [800, 800, 0]], path1.path, path1.time, 100, "../data/non_iter_navidi_3_sensors_100.mat")

    fig = plt.figure(figsize = (8, 8))
    ax1 = fig.add_subplot(111, projection='3d')
    ax1.scatter(path1.path[:, 0], path1.path[:, 1], path1.path[:, 2], 'green')
    plt.draw()
    plt.show()
    # ax1.plot3D(path1.position[0, 0], path1.position[0, 1], path1.position[0 , 2], 'b.-')
    # for i in np.arange(1,len(path1.path)+1):
    #     ax1.plot3D(path1.path[0:i, 0], path1.path[0:i, 1], path1.path[0:i, 2], 'green')
    #     plt.draw()
    #     plt.show()
        # prediction_position_iter = estimated_path_iter[i,:]
        # prediction_position_non_iter = estimated_path_non_iter(i,:)
        # ax3 = plot3(prediction_position_iter(1), prediction_position_iter(2), prediction_position_iter(3), 'r*')
        # ax3 = plot3(prediction_position_non_iter(1), prediction_position_non_iter(2), prediction_position_non_iter(3),
        #             'g*')
        # pause(sleep_duration)
        # delete(ax3)
        # ax3 = plot3(prediction_position_iter(1), prediction_position_iter(2), prediction_position_iter(3), 'r.')
        # ax3 = plot3(prediction_position_non_iter(1), prediction_position_non_iter(2), prediction_position_non_iter(3),
        #             'g.')


