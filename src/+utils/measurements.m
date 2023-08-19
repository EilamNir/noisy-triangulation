classdef measurements
    methods (Static)
        function [mean_err_iterative, mean_err_non_iterative, mean_err_deep_learn] = gen_monte_carlo(sensor_list, path, number_iterations)
            import estimation.iterative_estimator;
            import estimation.non_iterative_estimator_navidi;
            import estimation.deep_learning_estimator;
            % monte carlo
            SaveRuns = [];
            total_err_iter = zeros(size(path));
            total_err_non_iter = zeros(size(path));
            total_err_deep_learn = zeros(size(path));
            for j = 1:number_iterations
                for i = 1:size(sensor_list, 2)
                    sensor = sensor_list(i);
                    sensor.calculate_measurements(path);
                end
                iter_est = iterative_estimator(sensor_list, path(1,:));
                non_iter_est = non_iterative_estimator_navidi(sensor_list, path(1,:));
                deep_learn = deep_learning_estimator(sensor_list, path(1,:));
                
                estimated_path_iter = iter_est.estimate_path_by_distance();
                estimated_path_non_iter = non_iter_est.estimate_path_by_distance();
                estimated_path_deep_learn = deep_learn.estimate_path_by_distance();
                estimation_error_iter = (estimated_path_iter-path).^2;
                estimation_error_non_iter = (estimated_path_non_iter-path).^2;
                estimated_error_deep_learn = (estimated_path_deep_learn-path).^2;

                total_err_iter = total_err_iter + estimation_error_iter;
                total_err_non_iter = total_err_non_iter + estimation_error_non_iter;
                total_err_deep_learn = total_err_deep_learn + estimated_error_deep_learn;
            end
            mean_err_iterative = total_err_iter / number_iterations;
            mean_err_non_iterative = total_err_non_iter / number_iterations;
            mean_err_deep_learn = total_err_deep_learn / number_iterations;
        end
    end
end