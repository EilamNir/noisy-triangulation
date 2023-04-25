classdef iterative_estimator < handle
    properties
        sensor_list
        last_location
    end
    methods
        function obj = iterative_estimator(sensor_list, initial_location_guess)
            arguments
                sensor_list
                initial_location_guess
            end
            obj.sensor_list = sensor_list;
            obj.last_location = initial_location_guess
        end

        function distances = h(obj, sensor_locations, x_0)
            distances = ((x_0(1) - sensor_locations(:,1)).^2 + (x_0(2) - sensor_locations(:,2)).^2 + (x_0(3) - sensor_locations(:,3)).^2).^0.5;
        end

        function d_distances = dh(obj, sensor_locations, x_0)
            d_distances = x_0 ./ h(obj, sensor_locations, x_0);
        end

        function point = estimate_point_by_distances(obj, distances, sensor_locations, x_0)
            point = x_0
        end

        function estimated_path = estimate_path_by_distance(obj)
            arguments
                obj
            end

            all_distances = zeros([length(sensor_list) length(sensor_list(1).noisy_distances)]);
            sensor_locations = zeros([length(sensor_list) length(sensor_list(1).sensor_position)]);
            for i = 1:length(sensor_list)
                all_distances(i,:) = sensor_list(i).noisy_distances;
                sensor_locations(i,:) = sensor_list(i).sensor_position;
            end
            for i = 1:length(all_distances)
                current_point = obj.estimate_point_by_distances(all_distances(:,i), sensor_locations, obj.last_location)
                obj.last_location = current_point
            end
        end
    end
end
