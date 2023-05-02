classdef iterative_estimator < handle
    properties
        sensor_list
        last_location
        d_s
        Dd_s
    end
    methods
        function obj = iterative_estimator(sensor_list, initial_location_guess)
            arguments
                sensor_list
                initial_location_guess
            end
            obj.sensor_list = sensor_list;
            obj.last_location = initial_location_guess;
            
            % Create x variable to differentiate
            x = symmatrix('x', size(initial_location_guess));
            
            
            % Create symbolic sensor location to let us differentiate once and not for every sensor
            sens_loc = symmatrix('sens_loc', size(initial_location_guess));
            % Symbolic function to differentiate
            d = ((x-sens_loc) * (x-sens_loc).').^0.5;
            % Create symbolic functions that can get parameters (matrix symbolic functions can't use parameters)
            
            d_s(symmatrix2sym(sens_loc), symmatrix2sym(x)) = symmatrix2sym(d);
            Dd = diff(d,x);
            Dd_s(symmatrix2sym(sens_loc), symmatrix2sym(x)) = symmatrix2sym(Dd);
            obj.d_s = matlabFunction(d_s);
            obj.Dd_s = matlabFunction(Dd_s);
        end

        function [dist, Ddist] = get_distances(obj, sensor_locations, x0)
            % when calculating by hand:
            % distances = ((x0(1) - sensor_locations(:,1)).^2 + (x0(2) - sensor_locations(:,2)).^2 + (x0(3) - sensor_locations(:,3)).^2).^0.5;
            % d_distances = (x0 - sensor_locations) ./ distances;

            % Convert row to column vectors
            sensor_locations = sensor_locations';
            % Example of how vectors should look like:
            % sensor_locations = [0,0,0; 0,1,0; 1,0,0; -1,-1,0]';
            % x0 = [1, 1, 0];

            x0_c = num2cell(x0);

            % Create location to put output vector of symbols
            f = zeros([1 size(sensor_locations, 2)]);
            Df = zeros([size(x0,2) size(sensor_locations, 2)]);
            
            % Calculate function and derivative for each sensor
            for i = 1:length(f)
                sens_loc_c = num2cell(sensor_locations(:,i));
                f(i) = obj.d_s(sens_loc_c{:}, x0_c{:});
                Df(:,i) = obj.Dd_s(sens_loc_c{:}, x0_c{:});
            end
            % Convert vector of symbols to numbers
            dist = f';
            Ddist = Df';
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

        function estimated_path = estimate_path_by_distance(obj)
            arguments
                obj
            end

            all_distances = zeros([size(obj.sensor_list, 2) size(obj.sensor_list(1).noisy_distances, 1)]);
            sensor_locations = zeros([size(obj.sensor_list, 2) size(obj.sensor_list(1).sensor_position, 2)]);
            estimated_path = zeros([size(obj.sensor_list(1).noisy_distances, 1) size(obj.sensor_list(1).sensor_position, 2)]);
            for i = 1:size(obj.sensor_list, 2)
                all_distances(i,:) = obj.sensor_list(i).noisy_distances;
                sensor_locations(i,:) = obj.sensor_list(i).sensor_position;
            end
            % estimated_path = obj.estimate_point_by_distances(all_distances(:,1), sensor_locations, obj.last_location);
            f = waitbar(0, "please wait");
            for i = 1:size(all_distances, 2)
                current_point = obj.estimate_point_by_distances(all_distances(:,i), sensor_locations, obj.last_location);
                obj.last_location = current_point;
                estimated_path(i,:) = current_point;
                waitbar(i/size(all_distances, 2), f, "processing");
            end
            close(f);
        end
    end
end
