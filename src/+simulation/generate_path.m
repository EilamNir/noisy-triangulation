classdef generate_path
    properties
        heading
        position
        speed
        TimeRes
    end
    methods
        function obj = generate_path(initial_position, initial_heading, initial_speed, TimeRes)
        %generate_path - generate a path made out of intervals of different types
        %
        % Syntax: obj = generate_path(initial_position, initial_heading)
        %
        % generate a path of different intervals.
        % The path will start from initial_position (defaults to [0,0,0]), 
        % and will grow towards initial_heading (defaults to [0,0]).
        % The initial speed of the target is indicated at initial_speed (defaults to 50).
        % TimeRes defaults to 0.5.
            arguments
                initial_position = [0,0,0]
                initial_heading = [0,0]
                initial_speed = 50
                TimeRes = 0.5
            end
            obj.heading = initial_heading;
            obj.position = initial_position;
            obj.speed = initial_speed;
            obj.TimeRes = TimeRes;
        end

        function path_data = get_path_data(obj)
        %get_path_data - generate the full data of the path
        %
        % Syntax: path_data = get_path_data()
        %
        % Generate the true data of the path the target went trough.
        % Before calling this function, the user must add intervals to the path.
            path_data = 3;
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
                interval_duration uint32
                should_zero_pitch uint32 = 0
            end

            disp(should_zero_pitch);
        end
    end
end
