
import numpy as np

class iterative_estimator:
    def __init__(self, sensors, initial_guess_position):

        # Initialize parameters
        self.sensor_locations = sensors.sensor_locations
        self.distances = sensors.noisy_distances
        self.last_location = np.array(initial_guess_position)

        # Initialize functions
        self.dist_func = lambda sensor_pos, pos: (np.linalg.norm(pos - sensor_pos, 2, 1))
        self.dist_derivative_func = lambda sensor_pos, pos: ((pos - sensor_pos) / np.linalg.norm(pos - sensor_pos, 2, 1))

    def estimate_point_by_distance(self, distances, last_estimate, iteration_limit=5, precision_limit=0.01, epsilon=0.1):
        # Initialize parameters with correct dimensions
        last_estimate = last_estimate.T
        current_estimate = last_estimate
        y = distances.T


        # Iterate until we get close enough to the point
        for i in range(iteration_limit):
            # Get distance and derivative for current point
            h_x0 = self.dist_func(self.sensor_locations, current_estimate.T)
            H = self.dist_derivative_func(self.sensor_locations, current_estimate.T)

            # Make sure H isn't singular
            H = H + np.eye(H.shape[0]) * epsilon
            Z = np.linalg.inv(H.T @ H) @ H.T
            # Get better estimate
            current_estimate = Z @ (y - h_x0 + H @ current_estimate)

            # stop if we don't get closer
            if np.linalg.norm(last_estimate - current_estimate) < precision_limit:
                break
            last_estimate = current_estimate
        
        return current_estimate.T
    
    def estimate_path(self):
        estimated_path = np.zeros([self.distances.shape[0], self.sensor_locations.shape[1]])
        for i in range(self.distances.shape[0]):
            current_estimate = self.estimate_point_by_distance(self.distances[i,:], self.last_location)
            self.last_location = current_estimate
            estimated_path[i,:] = current_estimate
        return estimated_path
    
    def get_cov_err(self, true_path):
        path_cov_err = np.zeros(true_path.shape)
        for i in range(true_path.shape[0]):
            current_point = true_path[i,:]
            H = self.dist_derivative_func(self.sensor_locations, current_point)
            cov_vec = np.diag(np.linalg.inv(H.T @ H))
            path_cov_err[i,:] = cov_vec.T
        return path_cov_err
