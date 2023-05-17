classdef measurements
    methods (Static)
        function mean_err = gen_monte_carlo(sensor_list, path, number_iterations)
            import estimation.iterative_estimator;
            % monte carlo
            SaveRuns = [];
            total_err = zeros(size(path));
            for j = 1:number_iterations
                for i = 1:size(sensor_list, 2)
                    sensor = sensor_list(i);
                    sensor.calculate_measurements(path);
                end
                it = iterative_estimator(sensor_list, path(1,:));
                
                estimated_path = it.estimate_path_by_distance();
                estimation_error = (estimated_path-path).^2;

                total_err = total_err + estimation_error;
            end
            mean_err = total_err / number_iterations;
        end
    end
end