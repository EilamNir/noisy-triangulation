classdef generate_path < handle
    properties
        phi
        theta
        position
        speed
        acceleration
        TimeRes
        temp_should_test_curve % TODO: Remove me
        path
    end
    methods
        function obj = generate_path(initial_position, initial_heading, initial_speed, TimeRes)
        %generate_path - generate a path made out of intervals of different types
        %
        % Syntax: obj = generate_path(initial_position, initial_heading)
        %
        % generate a path of different intervals.
        % The path will start from initial_position (defaults to [0,0,0]), 
        % and will grow towards initial_heading (defaults to [phi=0,theta=0]).
        % The initial speed of the target is indicated at initial_speed (defaults to 50).
        % TimeRes defaults to 0.5.
            arguments
                initial_position = [0,0,0]
                initial_heading = [0,0]
                initial_speed = 50
                TimeRes = 0.5
            end
            obj.position = initial_position;
            obj.phi = initial_heading(1);
            obj.theta = initial_heading(2);
            obj.speed = initial_speed;
            obj.acceleration = 0;
            obj.TimeRes = TimeRes;
            obj.path = obj.position;
            obj.temp_should_test_curve = false; % TODO: Remove me
        end

        function path_data = get_path_data(obj, no_copy)
        %get_path_data - generate the full data of the path
        %
        % Syntax: path_data = get_path_data()
        %
        % Generate the true data of the path the target went trough.
        % Before calling this function, the user must add intervals to the path.
            arguments
                obj
                no_copy = true
            end

            if no_copy
                path_data = obj.path;
            else
                path_data = copy(obj.path);
            end
        end

        function add_straight_interval(obj, interval_duration, should_zero_pitch)
        %add_straight_interval - add a straight interval to the path
        %
        % Syntax: add_straight_interval(interval_duration)
        %
        % Add a straight interval to the path.
        % interval_duration is the total length of the path to add.
        % if should_zero_pitch is true, the straight interval will be only in the x-y plane.
            arguments
                obj
                interval_duration
                should_zero_pitch = false
            end
            
            if should_zero_pitch
               error("unimplemented") ;
            else
                obj.advance_path(interval_duration);
            end
        end

        function advance_path(obj, interval_duration)
        %advance_path - internal function to advance the path
        %
        % Syntax: advance_path(interval_duration)
        %
        % advance the path using the current position, speed and acceleration
            import utils.matrix_helpers.TransposeMatrix
            t = 0:obj.TimeRes:interval_duration;
            for i = t
                obj.position = obj.position + (obj.TimeRes * obj.speed .* TransposeMatrix(obj.phi, obj.theta))';
                obj.path(end+1,:) = obj.position;

                % TODO: Remove this, it's here for testing only
                if obj.temp_should_test_curve
                    obj.theta = obj.theta - obj.TimeRes*5;
                end

            end
        end
    end
end
