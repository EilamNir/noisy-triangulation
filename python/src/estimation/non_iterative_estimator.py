
import numpy as np

class non_iterative_estimator:
    def __init__(self, sensors, initial_guess_position=[0,0,0]):

        # Initialize parameters
        self.sensor_locations = sensors.sensor_locations
        self.distances = sensors.noisy_distances
        self.last_location = np.array(initial_guess_position)
        self.theta_bar = np.mean(self.sensor_locations, 0)
        self.X_star = 2*(self.sensor_locations - self.theta_bar)

    def estimate_point_by_distance(self, distances, epsilon=0.1):
        # Initialize parameters with correct dimensions
        Y = np.sum(np.square(self.sensor_locations - self.theta_bar), 1) - np.square(distances)
        Y_star = Y + self.X_star @ self.theta_bar.T
        point = np.linalg.inv((self.X_star.T@self.X_star) + np.eye(3)*epsilon)@self.X_star.T@Y_star
        
        return point
    
    def estimate_path(self):
        estimated_path = np.zeros([self.distances.shape[0], self.sensor_locations.shape[1]])
        for i in range(self.distances.shape[0]):
            current_estimate = self.estimate_point_by_distance(self.distances[i,:])
            self.last_location = current_estimate
            estimated_path[i,:] = current_estimate
        return estimated_path
    
    def get_cov_err(self, true_path):
        path_cov_err = np.zeros(true_path.shape)
        for i in range(true_path.shape[0]):
            cov_vec = np.diag(np.linalg.inv(self.X_star.T@self.X_star))
            path_cov_err[i,:] = cov_vec.T
        return path_cov_err
