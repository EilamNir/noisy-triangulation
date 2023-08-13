classdef noisy_sensor < handle
    properties
        sensor_position
        has_distance
        perfect_distances
        noisy_distances
        has_angle
        perfect_phis
        noisy_phis
        perfect_thetas
        noisy_thetas
        distance_noise_form
        distance_noise_sigma
        distance_noise_mu
        phi_noise_form
        phi_noise_sigma
        phi_noise_mu
        theta_noise_form
        theta_noise_sigma
        theta_noise_mu
    end
    methods
        function obj = noisy_sensor(sensor_position, options)
            arguments
                sensor_position = [0,0,0]

                options.has_distance = false
                options.distance_noise_form = 'Normal'
                options.distance_noise_mu = 0
                options.distance_noise_sigma = 0

                options.has_angle = false
                options.phi_noise_form = 'Normal'
                options.phi_noise_mu = 0
                options.phi_noise_sigma = 0
                options.theta_noise_form = 'Normal'
                options.theta_noise_mu = 0
                options.theta_noise_sigma = 0
            end
            obj.sensor_position = sensor_position;
            obj.has_distance = options.has_distance;
            obj.distance_noise_form = options.distance_noise_form;
            obj.distance_noise_mu = options.distance_noise_mu;
            obj.distance_noise_sigma = options.distance_noise_sigma;
            obj.perfect_distances = [];
            obj.noisy_distances = [];

            obj.has_angle = options.has_angle;
            obj.phi_noise_form = options.phi_noise_form;
            obj.phi_noise_mu = options.phi_noise_mu;
            obj.phi_noise_sigma = options.phi_noise_sigma;
            obj.theta_noise_form = options.theta_noise_form;
            obj.theta_noise_mu = options.theta_noise_mu;
            obj.theta_noise_sigma = options.theta_noise_sigma;
            obj.perfect_phis = [];
            obj.noisy_phis = [];
            obj.perfect_thetas = [];
            obj.noisy_thetas = [];
        end

        function calculate_measurements(obj, target_true_positions)
        %calculate_measurements - internal function to calculate measurements of sensor
        %
        % Syntax: calculate_measurements()
        %
        % calculate measurements including noise
            if obj.has_distance
                obj.perfect_distances = vecnorm(target_true_positions - obj.sensor_position,2,2);
                obj.noisy_distances = obj.perfect_distances + random(obj.distance_noise_form, obj.distance_noise_mu, obj.distance_noise_sigma, size(obj.perfect_distances));
            end
            if obj.has_angle
                obj.perfect_thetas = atan2(vecnorm(target_true_positions(:,1:2) - obj.sensor_position(:,1:2),2,2), target_true_positions(:,3) - obj.sensor_position(:,3));
                obj.noisy_thetas = obj.perfect_thetas + random(obj.theta_noise_form, obj.theta_noise_mu, obj.theta_noise_sigma, size(obj.perfect_thetas));
                obj.perfect_phis = atan2(target_true_positions(:,2) - obj.sensor_position(:,2), target_true_positions(:,1) - obj.sensor_position(:,1));
                obj.noisy_phis = obj.perfect_phis + random(obj.phi_noise_form, obj.phi_noise_mu, obj.phi_noise_sigma, size(obj.perfect_phis));
            end
        end
    end
end


% [1,1,1] .* ones(size(path1.path))