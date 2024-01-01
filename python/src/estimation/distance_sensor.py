
import numpy as np

class distance_sensors:
    def __init__(self, sensor_locations, sigma, outlier_percentage=None, outlier_sigma=None, initial_skip=10):
        self.sensor_locations = np.array(sensor_locations)
        # TODO: add a check to make sure it's always an array of sigmas for each sensor,
        # or change it to an array if it's a single value
        self.sigma = sigma
        self.mu = 0
        self.outlier_percentage = outlier_percentage
        self.outlier_sigma = outlier_sigma
        self.initial_skip = initial_skip

    def calculate_measurements(self, true_path):
        self.perfect_distances = np.zeros([true_path.shape[0], self.sensor_locations.shape[0]])
        for i, sensor_location in enumerate(self.sensor_locations):
            self.perfect_distances[:, i] = np.linalg.norm(true_path - sensor_location ,2, 1)
        noise = np.random.normal(self.mu, self.sigma, self.perfect_distances.shape)
        self.noisy_distances = self.perfect_distances + noise
        if self.outlier_percentage is not None:
            outliers = np.random.randint(0, 100, size=self.perfect_distances.shape)
            outliers = outliers > (100 - self.outlier_percentage)
            no_outlier_start = np.concatenate([np.ones((self.initial_skip,self.sensor_locations.shape[0])), np.zeros_like(outliers)[self.initial_skip:,:]], 0) < 0.5
            outliers = np.logical_and(no_outlier_start,outliers)
            outlier_noise = np.random.normal(self.mu, self.outlier_sigma, self.perfect_distances.shape)

            self.noisy_distances = self.perfect_distances + noise * (~outliers) + outlier_noise * outliers
            self.outlier_distances = np.any(outliers, 1)


# if __name__ == "__main__":
#     # minimal testing to check everything is working as intended
#     ds = distance_sensors([[100,0,0], [0,100,0], [0,0,100], [100,0,100]], 5)
#     ds.calculate_measurements(np.array([[0,0,0], [0,10,0], [0,20,0], [0,30,0], [0,100,0]]))
#     print(f"{ds.sensor_locations=}")
#     print(f"{ds.noisy_distances=}")
#     print(f"{ds.perfect_distances=}")
#     print(f"{ds.perfect_distances - ds.noisy_distances=}")
