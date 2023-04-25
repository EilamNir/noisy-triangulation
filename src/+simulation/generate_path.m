classdef generate_path < handle
    properties
        phi
        position
        speed_xy
        speed_z
        acceleration
        TimeRes
        path
    end
    methods
        function obj = generate_path(options)
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
                options.initial_position = [0,0,0]
                options.initial_phi = 0 % rad
                options.initial_speed_xy = 50 % m/s
                options.initial_speed_z = 10 % m/s
                options.TimeRes = 0.1 % s
            end
            obj.position = options.initial_position;
            obj.phi = options.initial_phi;
            obj.speed_xy = options.initial_speed_xy;
            obj.speed_z = options.initial_speed_z;
            obj.acceleration = 0;
            obj.TimeRes = options.TimeRes;
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
                options.phi = obj.phi
            end
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
                interval_duration % seconds
                rotation_speed % radians per second
                options.z_acceleration = 0 % m/s^2
            end
            obj.advance_path(interval_duration, options.z_acceleration, rotation_speed);
            
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
                obj.advance_path(interval_duration, rotation_speed*cos(angle), rotation_speed*sin(angle));
                
            end

        function advance_path(obj, interval_duration, z_acceleration, phi_rotation_speed)
        %advance_path - internal function to advance the path
        %
        % Syntax: advance_path(interval_duration)
        %
        % advance the path using the current position and speed
            import utils.matrix_helpers.TransposeMatrix2d
            t = obj.TimeRes:obj.TimeRes:interval_duration;
            for i = t
                obj.position(1:2) = obj.position(1:2) + (obj.TimeRes * obj.speed_xy .* TransposeMatrix2d(obj.phi))';
                obj.position(3) = obj.position(3) + (obj.TimeRes * obj.speed_z);
                obj.path(end+1,:) = obj.position;

                obj.phi = mod(obj.phi - obj.TimeRes * phi_rotation_speed, 2*pi);
            end
        end
    end
end
