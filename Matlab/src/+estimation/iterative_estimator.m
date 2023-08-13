classdef iterative_estimator < handle
    properties
        sensor_list
        last_location
        d_s
        Dd_s
        all_distances
        sensor_locations
        sensor_sigmas
    end
    methods
        function obj = iterative_estimator(sensor_list, initial_location_guess)
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

        function point = estimate_point_by_distances(obj, distances, sensor_locations, x0)
            current_estimate = x0';
            last_estimate = current_estimate;
            y = distances;
            for i = 1:5
                [h_x0, H] = obj.get_distances(sensor_locations, current_estimate');
                
                % To calculate how close the H matrix is to being singular, uncomment this section
                % [s, v, d] = svd(H'*H);
                % a = diag(v);
                % kappa = a(1)/a(end);
                % display(kappa);
                
                % Add a small value to diagonal of H to prevent it from being singular
                H = H + eye(size(H))*0.1;

                % Make sure the matrix we use can be inverted
                % (this is probably wasteful as it's probably quicker to just invert a 3x3 matrix in matlab than to test an if statement)
                % if (size(H,1) == size(H,2))
                %     Z = inv(H);
                % else
                %     Z = inv(H'*H)*H';
                % end
                % (the above is probably wasteful as it's probably quicker to just multiply a 3x3 matrix twice in matlab than to test an
                % if statement, so let's just multiply)
                Z = inv(H'*H)*H';
                
                % Get next estimate
                current_estimate = Z*(y - h_x0 + H*current_estimate);
                
                % temp_estimate = inv(H + eye(size(H))*0.1)*(y-h_x0) + current_estimate;
                % display(current_estimate);
                % display(temp_estimate);
                % display(current_estimate);

                % current_estimate = temp_estimate;
                if (norm(last_estimate - current_estimate) < 0.01)
                    break;
                end
                last_estimate = current_estimate;

                
            end
            % loss = (h_x0 - distances)';
            % display(loss);
            point = current_estimate';
        end

        function init_sensors(obj)
            all_distances = zeros([size(obj.sensor_list, 2) size(obj.sensor_list(1).noisy_distances, 1)]);
            sensor_locations = zeros([size(obj.sensor_list, 2) size(obj.sensor_list(1).sensor_position, 2)]);
            sensor_sigmas = zeros([size(obj.sensor_list, 2) 1]);
            for i = 1:size(obj.sensor_list, 2)
                all_distances(i,:) = obj.sensor_list(i).noisy_distances;
                sensor_locations(i,:) = obj.sensor_list(i).sensor_position;
                sensor_sigmas(i) = obj.sensor_list(i).distance_noise_sigma;
            end
            obj.all_distances = all_distances;
            obj.sensor_locations = sensor_locations;
            obj.sensor_sigmas = sensor_sigmas;
        end

        function estimated_path = estimate_path_by_distance(obj, options)
            arguments
                obj
                options.show_waitbar = false
            end
            show_waitbar = options.show_waitbar;

            all_distances = obj.all_distances;
            sensor_locations = obj.sensor_locations;
            estimated_path = zeros([size(obj.sensor_list(1).noisy_distances, 1) size(obj.sensor_list(1).sensor_position, 2)]);
            % estimated_path = obj.estimate_point_by_distances(all_distances(:,1), sensor_locations, obj.last_location);
            if show_waitbar
                f = waitbar(0, "please wait");
            end
            for i = 1:size(all_distances, 2)
                current_point = obj.estimate_point_by_distances(all_distances(:,i), sensor_locations, obj.last_location);
                obj.last_location = current_point;
                estimated_path(i,:) = current_point;
                if show_waitbar
                    waitbar(i/size(all_distances, 2), f, "processing");
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
