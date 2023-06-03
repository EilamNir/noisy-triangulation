classdef non_iterative_estimator < handle
    properties
        sensor_list
        last_location
        d_s
        Dd_s
        all_distances
        sensor_locations
        sensor_sigmas
        sensor_norm
    end
    methods
        function obj = non_iterative_estimator(sensor_list, initial_location_guess)
            arguments
                sensor_list
                initial_location_guess
            end
            obj.sensor_list = sensor_list;
            obj.last_location = initial_location_guess;
            obj.set_dist_functions();
            obj.init_sensors()
        end

        function set_dist_functions(obj)
            % % Symbolic functions:
            % % Create x variable to differentiate
            % x = symmatrix('x', size(initial_location_guess));
            % % Create symbolic sensor location to let us differentiate once and not for every sensor
            % sens_loc = symmatrix('sens_loc', size(initial_location_guess));
            % % Symbolic function to differentiate
            % d = ((x-sens_loc) * (x-sens_loc).').^0.5;
            % % Create symbolic functions that can get parameters (matrix symbolic functions can't use parameters)
            % d_s(symmatrix2sym(sens_loc), symmatrix2sym(x)) = symmatrix2sym(d);
            % Dd = diff(d,x);
            % Dd_s(symmatrix2sym(sens_loc), symmatrix2sym(x)) = symmatrix2sym(Dd);
            % obj.d_s = matlabFunction(d_s);
            % obj.Dd_s = matlabFunction(Dd_s);

            % Functions by hand:
            obj.d_s = @(sens_loc, x) vecnorm(x-sens_loc,2,2);
            obj.Dd_s = @(sens_loc, x) (x-sens_loc)./vecnorm(x-sens_loc,2,2);
        end

        function [dist, Ddist] = get_distances(obj, sensor_locations, x0)
            % If using the symbolic function:
            % dist = obj.d_s(sensor_locations(1,:), sensor_locations(2,:), sensor_locations(3,:), x0(1), x0(2), x0(3))';
            % Ddist = obj.Dd_s(sensor_locations(1,:), sensor_locations(2,:), sensor_locations(3,:), x0(1), x0(2), x0(3))';

            % Calculate function and derivative for each sensor
            dist = obj.d_s(sensor_locations, x0);
            Ddist = obj.Dd_s(sensor_locations, x0);
        end

        function point = estimate_point_by_distances_non_iterative(obj, distances, sensor_locations, x0)
            sensor_diff_matrix = sensor_locations(2:end,:)-sensor_locations(1,:);
            sensor_sub_norm = obj.sensor_norm(2:end)-obj.sensor_norm(1);
            y = distances(1)^2-distances(2:end).^2;
            point = 0.5*(inv(sensor_diff_matrix+eye([3,3])*0.1)*(y+sensor_sub_norm'));
%             m11 = [1 0 0;
%                    0 0 0;
%                    0 0 0];
%             m22 = [0 0 0
%                    0 1 0
%                    0 0 0];
%             m33 = [0 0 0
%                    0 0 0
%                    0 0 1];
%             v = [1;1;1];
%             point = 0.5*(inv(sensor_diff_matrix+eye([3,3])*0.1)*(y-...
%                     (m11*sensor_diff_matrix*sensor_sum_matrix*m11+...
%                     m22*sensor_diff_matrix*sensor_sum_matrix*m22+...
%                     m33*sensor_diff_matrix*sensor_sum_matrix*m33)*v));
        end

        function init_sensors(obj)
            for i = 1:size(obj.sensor_list, 2)
                obj.all_distances(i,:) = obj.sensor_list(i).noisy_distances;
                obj.sensor_locations(i,:) = obj.sensor_list(i).sensor_position;
                obj.sensor_sigmas(i) = obj.sensor_list(i).distance_noise_sigma;
                obj.sensor_norm(i) = norm(obj.sensor_list(i).sensor_position)^2;
            end
        end

        function estimated_path = estimate_path_by_distance(obj, options)
            arguments
                obj
                options.show_waitbar = false
            end
            show_waitbar = options.show_waitbar;

            if show_waitbar
                f = waitbar(0, "please wait");
            end
            for i = 1:size(obj.all_distances, 2)
                current_point = obj.estimate_point_by_distances_non_iterative(obj.all_distances(:,i), obj.sensor_locations, obj.last_location);
                obj.last_location = current_point;
                estimated_path(i,:) = current_point;
                if show_waitbar
                    waitbar(i/size(obj.all_distances, 2), f, "processing");
                end
            end
            if show_waitbar
                close(f);
            end
        end

        function path_cov_err = get_cov_err(obj, true_path)
            % path_cov_err is the expected [x_err, y_err, z_err] for each point on the path
            path_cov_err = zeros(size(true_path));

            for i = 1:size(true_path, 1)
                current_point = true_path(i,:);
                [h_x0, H] = obj.get_distances(obj.sensor_locations, current_point);
                cov_vec = diag(inv(H'*H));
                path_cov_err(i,:) = cov_vec';
            end
        end
    end
end
