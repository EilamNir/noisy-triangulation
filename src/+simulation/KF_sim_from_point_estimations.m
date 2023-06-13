close all; clear; clc;
%% simulation settings ****************************************************
SensorPos = [-5000,0,0; 400, -7400, 0; 800, 800, 0; 8000, 1000, 1000];
color_list = ['r', 'g', 'y', 'k', 'm'];
TargetPos = [0,0,5000];
TargetSpeed_xy = 50;
TargetSpeed_z = 10;
TargetRotSpeed = 3;
TimeRes = 0.5;
animation = false;

show_iterative = false;
show_non_iterative = true;
% show_iterative = true;
% show_non_iterative = false;
%%

% make the results reproducible by seeding the random number generator
rng(42);

% Create the path
import simulation.generate_path
path1 = generate_path("initial_phi", 0, "initial_speed_xy", TargetSpeed_xy, "initial_speed_z", TargetSpeed_z, "initial_position", TargetPos, "TimeRes", TimeRes);
path1.add_straight_interval(100);
path1.add_xy_turn_interval(90, -deg2rad(TargetRotSpeed));
path1.add_straight_interval(100);

true_path = path1.path;
path_time = path1.time;
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

% create point estimators
import estimation.iterative_estimator;
import estimation.non_iterative_estimator_navidi;

iter_est = iterative_estimator(sensor_list, true_path(1,:));
non_iter_est = non_iterative_estimator_navidi(sensor_list, true_path(1,:));

estimated_path_iter = iter_est.estimate_path_by_distance();
estimated_path_non_iter = non_iter_est.estimate_path_by_distance();

% get point estimation error
iter_point_estimation_err = sum((estimated_path_iter-true_path).^2, 2).^0.5;
non_iter_point_estimation_err = sum((estimated_path_non_iter-true_path).^2, 2).^0.5;

% use Kalman Filter on estimations
import estimation.kalman_filter_from_point_estimate;
sigma_a = 1;
sigma_v = 1;
non_diag_reduction_ratio = 2;
current_sample_reduction = 0.1;
iter_KF = kalman_filter_from_point_estimate(TimeRes, sigma_a, sigma_v, non_diag_reduction_ratio, current_sample_reduction);
non_iter_KF = kalman_filter_from_point_estimate(TimeRes, sigma_a, sigma_v, non_diag_reduction_ratio, current_sample_reduction);

estimated_path_iter_KF = iter_KF.run_filter_on_path(estimated_path_iter);
estimated_path_non_iter_KF = non_iter_KF.run_filter_on_path(estimated_path_non_iter);

% get KF estimation error
iter_KF_estimation_err = sum((estimated_path_iter_KF-true_path).^2, 2).^0.5;
non_iter_KF_estimation_err = sum((estimated_path_non_iter_KF-true_path).^2, 2).^0.5;

%% display path vs estimation

sleep_duration = 0.01;
figure();
subplot(2, 1, 1);
hold on 
grid minor
view([-37.5 30]);
xlim([min(true_path(:,1))-80 max(true_path(:,1))+80]);
ylim([min(true_path(:,2))-80 max(true_path(:,2))+80]);
zlim([min(true_path(:,3))-80 max(true_path(:,3))+80]);
xlabel('x'); ylabel('y'); zlabel('z');

for i = 1:size(SensorPos, 1)
    color = color_list(i);
    scatter3(SensorPos(i,1), SensorPos(i,2), SensorPos(i,3), 'filled', color);
end

subplot(2, 1, 2);
hold on 
grid minor
ax2 = 0;
ax3 = 0;
ax4 = 0;
ax5 = 0;
if animation
    subplot(2, 1, 1);
    ax1 = plot3(true_path(1:1,1), true_path(1:1,2), true_path(1:1,3), 'b.-');
    for i = 1:length(true_path)
        subplot(2, 1, 1);
        delete(ax1);

        % Plot the current point on the path as star
        ax1 = plot3(true_path(1:i,1), true_path(1:i,2), true_path(1:i,3), 'b.-');
        if show_iterative
            ax2_star = plot3(estimated_path_iter(i,1), estimated_path_iter(i,2), estimated_path_iter(i,3), 'r*');
            ax4_star = plot3(estimated_path_iter_KF(i,1), estimated_path_iter_KF(i,2), estimated_path_iter_KF(i,3), 'go'); 
        end
        
        if show_non_iterative
            ax3_star = plot3(estimated_path_non_iter(i,1), estimated_path_non_iter(i,2), estimated_path_non_iter(i,3), 'k*');
            ax5_star = plot3(estimated_path_non_iter_KF(i,1), estimated_path_non_iter_KF(i,2), estimated_path_non_iter_KF(i,3), 'm*'); 
        end

        % Plot the current error
        subplot(2, 1, 2);
        t = TimeRes:TimeRes:i*TimeRes;
        if show_iterative
            ax_err_2 = plot(t, iter_point_estimation_err(1:i), 'r-');
            ax_err_4 = plot(t, iter_KF_estimation_err(1:i), 'g-');
        end
        if show_non_iterative
            ax_err_3 = plot(t, non_iter_point_estimation_err(1:i), 'r-');
            ax_err_5 = plot(t, non_iter_KF_estimation_err(1:i), 'g-');
        end

        pause(sleep_duration);
        % Plot the current point on the path as dot
        subplot(2, 1, 1);
        if show_iterative
            delete([ax2 ax2_star ax4 ax4_star]);
            ax2 = plot3(estimated_path_iter(1:i,1), estimated_path_iter(1:i,2), estimated_path_iter(1:i,3), 'r.');
            ax4 = plot3(estimated_path_iter_KF(1:i,1), estimated_path_iter_KF(1:i,2), estimated_path_iter_KF(1:i,3), 'g.');
        end
        if show_non_iterative
            delete([ax3 ax3_star ax5 ax5_star]);
            ax3 = plot3(estimated_path_non_iter(1:i,1), estimated_path_non_iter(1:i,2), estimated_path_non_iter(1:i,3), 'k.');
            ax5 = plot3(estimated_path_non_iter_KF(1:i,1), estimated_path_non_iter_KF(1:i,2), estimated_path_non_iter_KF(1:i,3), 'm.'); 
        end
    end
else
    % plot path
    subplot(2, 1, 1);
    plot3(true_path(:,1), true_path(:,2), true_path(:,3), 'b.-');
    if show_iterative
        plot3(estimated_path_iter(:,1), estimated_path_iter(:,2), estimated_path_iter(:,3), 'r.');
        plot3(estimated_path_iter_KF(:,1), estimated_path_iter_KF(:,2), estimated_path_iter_KF(:,3), 'g.');
    end
    if show_non_iterative
        plot3(estimated_path_non_iter(:,1), estimated_path_non_iter(:,2), estimated_path_non_iter(:,3), 'k.');
        plot3(estimated_path_non_iter_KF(:,1), estimated_path_non_iter_KF(:,2), estimated_path_non_iter_KF(:,3), 'm.');
    end
    
    % plot error
    subplot(2, 1, 2);
    legend;
    t = TimeRes:TimeRes:size(iter_point_estimation_err, 1)*TimeRes;
    if show_iterative
        plot(t, iter_point_estimation_err, 'r-', 'DisplayName', 'point iterative estimation error (RMSE=' + string(sqrt(mean(iter_point_estimation_err.^2)))+")");
        plot(t, iter_KF_estimation_err, 'g-', 'DisplayName', 'KF iterative estimation error (RMSE=' + string(sqrt(mean(iter_KF_estimation_err.^2)))+")");
    end
    if show_non_iterative
        plot(t, non_iter_point_estimation_err, 'k-', 'DisplayName', 'point non estimation error (RMSE=' + string(sqrt(mean(non_iter_point_estimation_err.^2)))+")");
        plot(t, non_iter_KF_estimation_err, 'm-', 'DisplayName', 'KF non iterative estimation error (RMSE=' + string(sqrt(mean(non_iter_KF_estimation_err.^2)))+")");
    end
end
