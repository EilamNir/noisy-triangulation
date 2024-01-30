
import numpy as np

class kalman_filter_from_points_acc:
    def __init__(self, delta_t, sigma_a, sigma_v, non_diag_reduction_ratio=1, current_sample_reduction=1, initial_P_multiplier=1000):

        # Initialize parameters
        self.delta_t = delta_t
        self.sigma_a = sigma_a
        self.sigma_v = sigma_v
        self.non_diag_reduction_ratio = non_diag_reduction_ratio
        self.current_sample_reduction = current_sample_reduction
        self.initial_P_multiplier = initial_P_multiplier
        self.init_matrixes(delta_t)

    def init_matrixes(self, delta_t):
        # our state vector is x = [x, x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot].T
        one_dim_F = np.array([[1, delta_t, 0.5*np.square(delta_t)],[0, 1, delta_t], [0, 0, 1]])
        self.F = np.kron(np.eye(3), one_dim_F)

        one_dim_G = np.array([[0.5*np.square(delta_t), delta_t, 1]])
        self.G = np.kron(np.ones([1,3]), one_dim_G).T

        one_dim_H = np.array([1, 0, 0])
        self.H = np.kron(np.eye(3), one_dim_H)
        
        one_dim_v_to_X = np.array([0, 1, 0])
        self.v_to_X = np.kron(np.eye(3), one_dim_v_to_X)

        self.Q = self.G @ self.G.T * np.square(self.sigma_a)
    
    def get_initial_state(self, initial_path, n):
        x0 = initial_path[0,:,None]
        x_n = initial_path[n,:,None]
        v = (x_n - x0) / (self.delta_t * n)
        x = x0
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
        return X, P, self.H @ P @ self.H.T, (current_point - self.H @ X)
        

    def filter_path(self, noisy_path):
        estimated_path = np.copy(noisy_path)
        save_p = np.empty((noisy_path.shape[0], 9, 9))
        save_x = np.empty((noisy_path.shape[0], 9))
        save_L = np.empty((noisy_path.shape[0], 9))
        save_delta_x = np.empty((noisy_path.shape[0],3))
        avg_size = 2
        X = self.get_initial_state(noisy_path[0:avg_size+1, :], avg_size)
        P = self.initial_P_multiplier*np.eye(9)

        for i in range(noisy_path.shape[0]):
            X, P, L, delta_x = self.kalman_step(X, P, noisy_path[i,:,None])
            current_point = self.H @ X
            estimated_path[i,:] = current_point.T
            save_p[i, :, :] = P
            save_x[i, :] = X.T
            save_L[i, :] = L.reshape(1, -1)
            save_delta_x[i, :] = delta_x.reshape(1, -1)

        return estimated_path
        # return estimated_path, save_p, save_x, save_L, save_delta_x




# if __name__ == "__main__":
#     kf = kalman_filter_from_points(0.5, 1, 100, 2, 1)
#     print(f"{kf.F=} \n{kf.G=} \n{kf.H=} \n{kf.v_to_X=} \n{kf.Q=}")
#     print(f"{kf.F.shape=} \n{kf.G.shape=} \n{kf.H.shape=} \n{kf.v_to_X.shape=} \n{kf.Q.shape=}")
#     filtered_path = kf.filter_path(np.array([[1,1,1], [1,1,2], [2,2,3], [2,2,3], [2,2,4], [2,2,5]]))
#     print(f"{filtered_path=}")
