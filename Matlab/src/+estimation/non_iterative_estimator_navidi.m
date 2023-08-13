classdef non_iterative_estimator_navidi < handle
    properties
        sensor_list
        last_location
        all_distances
        sensor_locations
        theta_bar
        X_star
    end
    methods
        function obj = non_iterative_estimator_navidi(sensor_list, initial_location_guess)
            arguments
                sensor_list
                initial_location_guess
            end
            obj.sensor_list = sensor_list;
            obj.last_location = initial_location_guess;
            obj.init_sensors();
        end

        function point = estimate_point_by_distances_non_iterative(obj, distances, sensor_locations, x0)
            % implementation of Navidi, W., Murphy Jr, W. S., & Hereman, W. (1998). Statistical methods in surveying by trilateration. Computational statistics & data analysis, 27(2), 209-227.
            Y = sum((sensor_locations - obj.theta_bar).^2, 2) - distances.^2;
            Y_star = Y + obj.X_star * obj.theta_bar';
            point = inv(obj.X_star'*obj.X_star + eye([3,3])*0.1)*obj.X_star'*Y_star;
        end

        function init_sensors(obj)
            for i = 1:size(obj.sensor_list, 2)
                obj.all_distances(i,:) = obj.sensor_list(i).noisy_distances;
                obj.sensor_locations(i,:) = obj.sensor_list(i).sensor_position;
            end
            obj.theta_bar = mean(obj.sensor_locations, 1);
            obj.X_star = 2*(obj.sensor_locations - obj.theta_bar);
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
                cov_vec = diag(inv(obj.X_star'*obj.X_star));
                path_cov_err(i,:) = cov_vec';
            end
        end
    end
end
