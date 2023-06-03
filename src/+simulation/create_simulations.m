classdef create_simulations
    methods (Static)
        function save_simulation_MC(SensorPos_in, path, path_time_in, MC_iterations, file_path)
            true_path = path;
            SensorPos = SensorPos_in;
            path_time = path_time_in;
            sensor_dist_sigma = 15;
            
            % create sensors
            import simulation.noisy_sensor
            sensor_list(1,1) = noisy_sensor(SensorPos(1,:), "has_distance", true, "has_angle", false, "distance_noise_sigma", sensor_dist_sigma);
            for i = 2:size(SensorPos,1)
                sensor = noisy_sensor(SensorPos(i,:), "has_distance", true, "has_angle", false, "distance_noise_sigma", sensor_dist_sigma);
                sensor_list(end+1) = sensor;
            end

            % sensor sampling 
            for i = 1:size(sensor_list, 2)
                sensor = sensor_list(i);
                sensor.calculate_measurements(true_path);
            end

            % monte carlo
            import utils.measurements.gen_monte_carlo;
            [MC_MSE_iter, MC_MSE_non_iter] = gen_monte_carlo(sensor_list, true_path, MC_iterations);

            % test estimators
            import estimation.iterative_estimator;
            import estimation.non_iterative_estimator;

            iter_est = iterative_estimator(sensor_list, true_path(1,:));
            non_iter_est = non_iterative_estimator(sensor_list, true_path(1,:));

            estimated_path_iter = iter_est.estimate_path_by_distance();
            estimated_path_non_iter = non_iter_est.estimate_path_by_distance();

            % get cov error
            cov_mat_iter = iter_est.get_cov_err(true_path);
            cov_MSE_iter = cov_mat_iter * sensor_dist_sigma^2;
            cov_mat_non_iter = iter_est.get_cov_err(true_path);
            cov_MSE_non_iter = cov_mat_non_iter * sensor_dist_sigma^2;

            save(file_path, "true_path", "path_time", "estimated_path_iter", "estimated_path_non_iter", "MC_MSE_iter", "MC_MSE_non_iter", "cov_mat_iter", "cov_MSE_iter", "cov_mat_non_iter", "cov_MSE_non_iter", "SensorPos")
        end
    end
end