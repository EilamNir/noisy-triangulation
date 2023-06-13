classdef kalman_filter_from_point_estimate < handle
    properties
        delta_t
        sigma_a
        sigma_v
        F
        G
        H
        v_to_X
        Q
        non_diag_reduction_ratio
        current_sample_reduction
    end
    methods
        function obj = kalman_filter_from_point_estimate(delta_t, sigma_a, sigma_v, non_diag_reduction_ratio, current_sample_reduction)
            obj.delta_t = delta_t;
            obj.sigma_a = sigma_a;
            obj.sigma_v = sigma_v;
            obj.non_diag_reduction_ratio = non_diag_reduction_ratio;
            obj.current_sample_reduction = current_sample_reduction;
            obj.init_matrixes(delta_t);
        end

        function init_matrixes(obj, delta_t)
            % our state vector is x = [x; x_dot; y; y_dot; z; z_dot]
            one_dim_F = [1 delta_t; 0 1];
            one_dim_F_cell = repmat({one_dim_F}, 1, 3);
            obj.F = blkdiag(one_dim_F_cell{:});

            one_dim_G = [0.5*delta_t^2 ; delta_t];
            obj.G = repmat(one_dim_G, 3, 1);

            one_dim_H = [1 0];
            one_dim_H_cell = repmat({one_dim_H}, 1, 3);
            obj.H = blkdiag(one_dim_H_cell{:});
            
            one_dim_v_to_X = [0 1];
            one_dim_v_to_X_cell = repmat({one_dim_v_to_X}, 1, 3);
            obj.v_to_X = blkdiag(one_dim_v_to_X_cell{:});

            obj.Q = obj.G * obj.G' * obj.sigma_a^2;
        end

        function [X] = get_initial_state(obj, x1, x2)
            % initial location and speed from two first estimated points
            v = (x2' - x1') / obj.delta_t;
            x = (x2' + x1') / 2;
            % v = [50; 0; 10];
            % x = [0; 0; 5000];
            X = obj.v_to_X' * v + obj.H' * x;
        end

        function [X, P] = kalman_step(obj, last_X, last_P, current_measurement)
            P = obj.F * last_P * obj.F' + obj.Q;
            L_k = P * obj.H' / (obj.H * P * obj.H' + obj.sigma_v^2);
            % X = obj.F * last_X + obj.G * normrnd(0,1); % Why do we need this normrnd(0,1)?
            X = obj.F * last_X;
            % X = X + L_k * (current_measurement' - obj.H * X);
            X = X + obj.current_sample_reduction * L_k * (current_measurement' - obj.H * X);
            P = P - L_k * obj.H * P;
            % reduce anything in P that is not on the diagonal
            P = (diag(diag(P)) * (obj.non_diag_reduction_ratio - 1) + P) / obj.non_diag_reduction_ratio;
        end

        function estimated_path = run_filter_on_path(obj, initial_estimated_path)
            % Use the first 2 points for initial location and position
            estimated_path(1,:) = initial_estimated_path(1,:);
            estimated_path(2,:) = initial_estimated_path(2,:);
            X = obj.get_initial_state(initial_estimated_path(1,:), initial_estimated_path(2,:));
            P = eye(6);

            % We start from point 3 as we treat the first 2 as initial conditions
            for i = 3:size(initial_estimated_path, 1)
                [X,P] = kalman_step(obj, X, P, initial_estimated_path(i,:));
                current_point = obj.H * X;
                estimated_path(i,:) = current_point';
            end
        end

        function path_cov_err = get_cov_err(obj, true_path)
            % path_cov_err is the expected [x_err, y_err, z_err] for each point on the path
            path_cov_err = zeros(size(true_path));

            for i = 1:size(true_path, 1)
                current_point = true_path(i,:);
                
                cov_vec = diag(inv(obj.X_star'*obj.X_star));
                path_cov_err(i,:) = cov_vec';
            end
        end
    end
end
