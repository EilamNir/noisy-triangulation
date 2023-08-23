
import numpy as np

class kalman_filter_from_points:
    def __init__(self, delta_t, sigma_a, sigma_v, non_diag_reduction_ratio, current_sample_reduction):

        # Initialize parameters
        self.delta_t = delta_t
        self.sigma_a = sigma_a
        self.sigma_v = sigma_v
        self.non_diag_reduction_ratio = non_diag_reduction_ratio
        self.current_sample_reduction = current_sample_reduction
        self.init_matrixes(delta_t)

    def init_matrixes(self, delta_t):
        # our state vector is x = [x, x_dot, y, y_dot, z, z_dot].T
        one_dim_F = np.array([[1, delta_t],[0, 1]])
        self.F = np.kron(np.eye(3), one_dim_F)

        one_dim_G = np.array([[0.5*np.square(delta_t), delta_t]])
        self.G = np.kron(np.ones([1,3]), one_dim_G).T

        one_dim_H = np.array([1, 0])
        self.H = np.kron(np.eye(3), one_dim_H)
        
        one_dim_v_to_X = np.array([0, 1])
        self.v_to_X = np.kron(np.eye(3), one_dim_v_to_X)

        self.Q = self.G @ self.G.T * np.square(self.sigma_a)
    
    def get_initial_state(self, x0, x1):
        # v = (x1.T - x0.T) / self.delta_t
        # x = (x1.T + x0.T) / 2
        v = np.array([[50, 0, 10]]).T
        x = np.array([[0, 0, 5000]]).T
        X = self.v_to_X.T @ v + self.H.T @ x
        return X
    
    def kalman_step(self, last_X, last_P, current_point):
        P = self.F @ last_P @ self.F.T + self.Q
        L_k = P @ self.H.T @ np.linalg.inv(self.H @ P @ self.H.T + np.eye(3) * np.square(self.sigma_v))
        X = self.F @ last_X
        X = X + self.current_sample_reduction * L_k @ (current_point - self.H @ X)
        P = P - L_k @ self.H @ P
        # reduce anything in P that is not on the diagonal
        P = (np.diag(np.diag(P)) * (self.non_diag_reduction_ratio - 1) + P) / self.non_diag_reduction_ratio
        return X, P
        

    def filter_path(self, noisy_path):
        estimated_path = np.copy(noisy_path)
        X = self.get_initial_state(noisy_path[0,:,None], noisy_path[1,:,None])
        P = np.eye(6)

        for i in range(2, noisy_path.shape[0]):
            X, P = self.kalman_step(X, P, noisy_path[i,:,None])
            current_point = self.H @ X
            estimated_path[i,:] = current_point.T

        return estimated_path




if __name__ == "__main__":
    kf = kalman_filter_from_points(0.5, 1, 100, 2, 1)
    print(f"{kf.F=} \n{kf.G=} \n{kf.H=} \n{kf.v_to_X=} \n{kf.Q=}")
    print(f"{kf.F.shape=} \n{kf.G.shape=} \n{kf.H.shape=} \n{kf.v_to_X.shape=} \n{kf.Q.shape=}")
    filtered_path = kf.filter_path(np.array([[1,1,1], [1,1,2], [2,2,3], [2,2,3], [2,2,4], [2,2,5]]))
    print(f"{filtered_path=}")
