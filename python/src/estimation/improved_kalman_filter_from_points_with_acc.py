
import numpy as np
import torch.nn as nn
import torch

class improved_kalman_filter_from_points_acc:
    def __init__(self, delta_t, sigma_a, sigma_v, non_diag_reduction_ratio=1, current_sample_reduction=1):
        # Initialize parameters
        self.delta_t = delta_t
        self.sigma_a = sigma_a
        self.sigma_v = sigma_v
        self.non_diag_reduction_ratio = non_diag_reduction_ratio
        self.current_sample_reduction = current_sample_reduction
        self.init_matrixes(delta_t)

    def init_matrixes(self, delta_t):
        # our state vector is x = [x, x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot].T
        one_dim_F = np.array([[1, delta_t, 0.5 * np.square(delta_t)], [0, 1, delta_t], [0, 0, 1]])
        self.F = np.kron(np.eye(3), one_dim_F)

        one_dim_G = np.array([[0.5 * np.square(delta_t), delta_t, 1]])
        self.G = np.kron(np.ones([1, 3]), one_dim_G).T

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

    def kalman_step(self, current_point, last_p, last_x, last_L, last_delta_x, model):

        P = self.F @ last_p @ self.F.T + self.Q
        L_k = P @ self.H.T @ np.linalg.inv(self.H @ P @ self.H.T + np.eye(3) * np.square(self.sigma_v))
        X = self.F @ last_x
        x_new = X + self.current_sample_reduction * L_k @ (current_point - self.H @ X)
        p_new = P - L_k @ self.H @ P
        # reduce anything in P that is not on the diagonal
        p_new = (np.diag(np.diag(p_new)) * (self.non_diag_reduction_ratio - 1) + p_new) / self.non_diag_reduction_ratio
        L_new = self.H @ P @ self.H.T
        delta_x_new = (current_point - self.H @ X)
        # mahal =  sqrt(((current_point - self.H @ X).T * np.linalg.inv(self.H @ P @ self.H.T + np.eye(3) * np.square(self.sigma_v))*(current_point - self.H @ X))/len(current_point)) if > 3 x_new = x
        input = np.concatenate((current_point.reshape(1,-1), np.reshape(last_p, (1,-1)), last_x.reshape(1,-1),
                                last_L.reshape(1,-1), last_delta_x.reshape(1,-1), np.reshape(p_new, (1,-1)),
                                x_new.reshape(1,-1), L_new.reshape(1,-1), delta_x_new.reshape(1,-1)), 1)

        outputs = model(torch.from_numpy(input).unsqueeze(0).float())
        if outputs[0,0].item() > 0.2:
            x_new = X
            p_new = P
            p_new = (np.diag(np.diag(p_new)) * (
                        self.non_diag_reduction_ratio - 1) + p_new) / self.non_diag_reduction_ratio

        return x_new, p_new, L_new, delta_x_new

    def filter_path(self, noisy_path):
        estimated_path = np.copy(noisy_path)
        save_p = np.empty((noisy_path.shape[0], 9, 9))
        save_x = np.empty((noisy_path.shape[0], 9))
        save_L = np.empty((noisy_path.shape[0], 9))
        save_delta_x = np.empty((noisy_path.shape[0], 3))
        avg_size = 2
        X = self.get_initial_state(noisy_path[0:avg_size + 1, :], avg_size)
        P = 1000 * np.eye(9)
        L = np.zeros((9,1))
        delta_x = np.zeros((3,1))

        path = "best_state_dict.pt"
        model = state_estimat(d_in=4, num_classes=1)
        model.load_state_dict(torch.load(path))
        model.eval()

        for i in range(noisy_path.shape[0]):
            X, P, L, delta_x = self.kalman_step(noisy_path[i, :, None], P, X, L, delta_x, model)
            current_point = self.H @ X
            estimated_path[i, :] = current_point.T
            save_p[i, :, :] = P
            save_x[i, :] = X.T
            save_L[i, :] = L.reshape(1, -1)
            save_delta_x[i, :] = delta_x.reshape(1, -1)

        return estimated_path


class state_estimat(nn.Module):
    def __init__(self, d_in, num_classes):
        # initialzing the parent object (important!)
        super(state_estimat, self).__init__()
        # Create a pipeline - a sequence of layers
        self.pipe = torch.nn.Sequential(
            nn.Conv1d(1,32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv1d(32, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(6624, num_classes))

    def forward(self, x):
        return self.pipe(x)