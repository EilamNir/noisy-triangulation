classdef generate_path < handle
    properties
        theta
        phi
        position
        speed
        acceleration
        TimeRes
        path
    end
    methods
        function obj = generate_path(initial_position, initial_theta, initial_phi, initial_speed, TimeRes)
        %generate_path - generate a path made out of intervals of different types
        %
        % Syntax: obj = generate_path(initial_position, initial_theta, initial_phi, initial_speed, TimeRes)
        %
        % Generate a path of different intervals.
        % The path will start from initial_position (defaults to [0,0,0]), 
        % and will grow towards [theta, phi] (defaults to [theta=0, phi=0]).
        % The initial speed of the target is indicated at initial_speed (defaults to 50).
        % TimeRes defaults to 0.5.
            arguments
                initial_position = [0,0,0]
                initial_theta = 90
                initial_phi = 0
                initial_speed = 50
                TimeRes = 0.5
            end
            obj.position = initial_position;
            obj.theta = initial_theta;
            obj.phi = initial_phi;
            obj.speed = initial_speed;
            obj.acceleration = 0;
            obj.TimeRes = TimeRes;
            obj.path = obj.position;
        end

        function add_straight_interval(obj, interval_duration, options)
        %add_straight_interval - add a straight interval to the path
        %
        % Syntax: add_straight_interval(interval_duration)
        %
        % Add a straight interval to the path.
        % interval_duration is the total length of the path to add.
        % If any of theta or phi are given, the heading will change before the interval.
            arguments
                obj
                interval_duration
                options.theta = obj.theta
                options.phi = obj.phi
            end
            obj.theta = options.theta;
            obj.phi = options.phi;
            obj.advance_path(interval_duration, 0, 0);
        end
        
        function add_xy_turn_interval(obj, interval_duration, rotation_speed, options)
        %add_xy_turn_interval - add a turn in the x-y plane, with a constant theta
        %
        % Syntax: add_xy_turn_interval(obj, rotation_speed)
        %
        % Add a turn interval, which has a constant radius, and can climb or descend in the z axis.
            arguments
                obj
                interval_duration
                rotation_speed
                options.override_theta = true
                options.theta = 90
            end
            if options.override_theta
                obj.theta = options.theta;
            end
            obj.advance_path(interval_duration, 0, rotation_speed);
            
        end

        function add_3d_turn_interval(obj, interval_duration, rotation_speed, angle)
            %add_3d_turn_interval - add a turn in 3 dimensions
            %
            % Syntax: add_3d_turn_interval(obj, interval_duration, rotation_speed, angle)
            %
            % Add a turn interval, which has a constant radius.
            % The angle controls the direction of the turn.
            % angle of 0 means pitching up, and angle of 90 means turning fully right.
                arguments
                    obj
                    interval_duration
                    rotation_speed
                    angle
                end
                obj.advance_path(interval_duration, rotation_speed*cosd(angle), rotation_speed*sind(angle));
                
            end

        function advance_path(obj, interval_duration, theta_rotation_speed, phi_rotation_speed)
        %advance_path - internal function to advance the path
        %
        % Syntax: advance_path(interval_duration)
        %
        % advance the path using the current position and speed
            import utils.matrix_helpers.TransposeMatrix
            t = 0:obj.TimeRes:interval_duration;
            for i = t
                obj.position = obj.position + (obj.TimeRes * obj.speed .* TransposeMatrix(obj.phi, obj.theta))';
                obj.path(end+1,:) = obj.position;

                obj.theta = mod(obj.theta - obj.TimeRes * theta_rotation_speed, 360);
                obj.phi = mod(obj.phi - obj.TimeRes * phi_rotation_speed, 360);
            end
        end
    end
end
