
import numpy as np

class distance_sensors:
    def __init__(self, sensor_locations, sigma):
        self.sensor_locations = np.array(sensor_locations)
        # TODO: add a check to make sure it's always an array of sigmas for each sensor,
        # or change it to an array if it's a single value
        self.sigma = sigma
        self.mu = 0

    def calculate_measurements(self, true_path):
        self.perfect_distances = np.zeros([true_path.shape[0], self.sensor_locations.shape[0]])
        for i, sensor_location in enumerate(self.sensor_locations):
            self.perfect_distances[:, i] = np.linalg.norm(true_path - sensor_location ,2, 1)
        self.noisy_distances = self.perfect_distances + np.random.normal(self.mu, self.sigma, self.perfect_distances.shape)
        

if __name__ == "__main__":
    # minimal testing to check everything is working as intended
    ds = distance_sensors([[100,0,0], [0,100,0], [0,0,100], [100,0,100]], 5)
    ds.calculate_measurements(np.array([[0,0,0], [0,10,0], [0,20,0], [0,30,0], [0,100,0]]))
    print(f"{ds.sensor_locations=}")
    print(f"{ds.noisy_distances=}")
    print(f"{ds.perfect_distances=}")
    print(f"{ds.perfect_distances - ds.noisy_distances=}")
