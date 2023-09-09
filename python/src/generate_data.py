import numpy as np
from simulation.generate_path import generate_path
import matplotlib.pyplot as plt
from estimation.distance_sensor import distance_sensors
import random
import time
import sys
from estimation.kalman_filter_from_points_with_acc import kalman_filter_from_points_acc
from estimation.dual_iterative_non_iterative_estimator import iter_non_iter_estimator
import pickle
from tqdm import tqdm

def generate_data(output_filename, run_number):
    sensor_amount = 3
    past_sample_len = 10
    filled_len = 0
    time_res = 0.5
    
    # dict to fill with data
    train_data = {
        "sensor_locations":np.zeros((sensor_amount,3,past_sample_len,run_number*(200 * int(1/time_res)))),
        "true_path":np.zeros((1,3,past_sample_len,run_number*(200 * int(1/time_res)))),
        "dual_estimate_path":np.zeros((1,3,past_sample_len,run_number*(200 * int(1/time_res)))),
        "kf_estimate_path":np.zeros((1,3,past_sample_len,run_number*(200 * int(1/time_res)))),
        "state_key":np.zeros((run_number*(200 * int(1/time_res)),1)),
    }

    # generate the data
    for k in tqdm (range (run_number), desc="generating data..."):
        target_initial_pos = np.random.randint(-7000, 7000, size=(1, 3))[0]
        sensors_pos = np.random.randint(-7000, 7000, size=(sensor_amount, 3))[:,:]
        target_speed_xy = 50
        target_speed_z = 10
        target_rot_speed = 3

        path1 = generate_path(np.deg2rad(np.random.randint(0,360,size=1)[0]), target_speed_xy, target_speed_z, target_initial_pos, time_res)
        path1.add_straight_interval(np.random.randint(10,100,size=1)[0])
        path1.add_xy_turn_interval(np.random.randint(10,100,size=1)[0], -random.choice([-1, 1])*np.deg2rad(target_rot_speed))
        total_len = len(path1.path)

        # create noisy sensors
        sensors = distance_sensors(sensors_pos, 20)
        sensors.calculate_measurements(path1.path)
        sigma_a = 1
        sigma_v = 100
        kf_acc = kalman_filter_from_points_acc(time_res, sigma_a, sigma_v, non_diag_reduction_ratio=2)
        dual_est = iter_non_iter_estimator(sensors)
        dual_est_path = dual_est.estimate_path()
        kf_path_acc = kf_acc.filter_path(dual_est_path)

        for i in range(total_len - past_sample_len + 1):
            train_data["sensor_locations"][:,:,:,filled_len+i] = np.repeat(sensors.sensor_locations.reshape(sensor_amount,3,1), past_sample_len, axis=2)
            train_data["true_path"][:,:,:,filled_len+i] = path1.path[i:i+past_sample_len,:].T.reshape(1,3,past_sample_len)
            train_data["dual_estimate_path"][:,:,:,filled_len+i] = kf_path_acc[i:i+past_sample_len,:].T.reshape(1,3,past_sample_len)
            train_data["kf_estimate_path"][:,:,:,filled_len+i] = dual_est_path[i:i+past_sample_len,:].T.reshape(1,3,past_sample_len)
        train_data["state_key"][filled_len:filled_len+total_len-past_sample_len+1,:] = path1.state_key[past_sample_len-1:]
        filled_len += total_len-past_sample_len+1
    
    train_data["sensor_locations"] = train_data["sensor_locations"][:,:,:,:filled_len]
    train_data["true_path"] = train_data["true_path"][:,:,:,:filled_len]
    train_data["dual_estimate_path"] = train_data["dual_estimate_path"][:,:,:,:filled_len]
    train_data["kf_estimate_path"] = train_data["kf_estimate_path"][:,:,:,:filled_len]
    train_data["state_key"] = train_data["state_key"][:filled_len,:]

    # save data to file
    print(f"saving data to {output_filename}")

    with open(output_filename, "wb") as f:
        pickle.dump(train_data, f)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <output_filename> <run_number>")
    else:
        generate_data(sys.argv[1], int(sys.argv[2]))