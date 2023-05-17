classdef create_simulations
    methods (Static)
        function [true_path, path_time, estimated_path, MC_MSE, cov_mat, cov_MSE, SensorPos] = save_simulation_MC(SensorPos_in, path, path_time_in, file_path)
            true_path = path;
            SensorPos = SensorPos_in;
            path_time = path_time_in;
            MC_iterations = 100;
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
            MC_MSE = gen_monte_carlo(sensor_list, true_path, MC_iterations);

            % test iterative estimator
            import estimation.iterative_estimator;

            it = iterative_estimator(sensor_list, true_path(1,:));

            estimated_path = it.estimate_path_by_distance();

            % get cov error
            cov_mat = it.get_cov_err(true_path);
            cov_MSE = cov_mat * sensor_dist_sigma^2;

            save(file_path, "true_path", "path_time", "estimated_path", "MC_MSE", "cov_mat", "cov_MSE", "SensorPos")
        end

        function [true_path, path_time, estimated_path, MC_MSE, cov_mat, cov_MSE, SensorPos] = load_simulation_MC(file_path)
            load(file_path, "true_path", "path_time", "estimated_path", "MC_MSE", "cov_mat", "cov_MSE", "SensorPos")
        end
    end
end