close all; clear; clc;
%% simulation settings ****************************************************
SensorPos = [-5000,0,0; 400, -7400, 0; 800, 800, 0];
TargetPos = [0,0,1000];
TargetSpeed_xy = 50;
TargetSpeed_z = 10;
TargetRotSpeed = 3;
TimeRes = 2;
SimulationDuration = 150;
TargetTheta = 90;
TargetPhi = 45;

% make the results reproducible by seeding the random number generator
rng(42);

%% generate a path
import simulation.generate_path

%% test straight lines and xy turns
path1 = generate_path("initial_phi", 0, "initial_speed_xy", TargetSpeed_xy, "initial_speed_z", TargetSpeed_z, "initial_position", TargetPos, "TimeRes", TimeRes);
path1.add_straight_interval(100);
%path1.add_xy_turn_interval(10, -pi*3/(2 * 10));
path1.add_xy_turn_interval(90, -deg2rad(TargetRotSpeed));
path1.add_straight_interval(100);

% test sensors
import simulation.noisy_sensor

%% create sensors
sensor_list = [];
for i = 1:size(SensorPos,1)
    dataset = noisy_sensor(SensorPos(i,:), "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);
    assignin('base',['sensor_distance' num2str(i)], dataset);
    sensor_list = cat(2,sensor_list,eval(['sensor_distance' num2str(i)]));
%     dataset = noisy_sensor(sensor_angle1_pos, "has_distance", false, "has_angle", true, "theta_noise_sigma", 5, "theta_noise_sigma", 5);
%     assignin('base',['sensor_angle' num2str(i)], dataset);
%     sensor_list = cat(2,sensor_list,eval(['sensor_angle' num2str(i)]));
end

%% sensor sampling 
color_list = ['r', 'g', 'y', 'k'];

for i = 1:size(sensor_list, 2)
    sensor = sensor_list(i);
    sensor.calculate_measurements(path1.path);
end


%% monte carlo
SaveRuns = [];
for j = 1:100
    for i = 1:size(sensor_list, 2)
        sensor = sensor_list(i);
        sensor.calculate_measurements(path1.path);
    end
    
    import estimation.iterative_estimator;
    
    it = iterative_estimator(sensor_list, path1.path(1,:));
    
    estimated_path = it.estimate_path_by_distance();
    
    SaveRuns = cat(3,SaveRuns,estimated_path-path1.path);
end

var = sum(SaveRuns.^2,3)/size(SaveRuns,3); 

%% test iterative estimator
import estimation.iterative_estimator;

it = iterative_estimator(sensor_list, path1.path(1,:));

estimated_path = it.estimate_path_by_distance();

%% get cov error
path_cov_err = it.get_cov_err(path1.path);
figure
hold on
grid minor
xlabel('time');
ylabel('error');
legend('Location','east');
plot(path1.time, path_cov_err(:,1) / 3, '--','DisplayName', 'x cov / 3', "color", "r")
plot(path1.time, var(:,1), 'DisplayName', 'x MC', "color", "#A2142F")
plot(path1.time, path_cov_err(:,2) / 3, '--','DisplayName', 'y cov / 3', "color", "g")
plot(path1.time, var(:,2), 'DisplayName', 'y MC', "color", "#77AC30")
figure
hold on
grid minor
xlabel('time');
ylabel('error');
legend('Location','east');
plot(path1.time, path_cov_err(:,3) / 3, '--','DisplayName', 'z cov / 3', "color", "b")
plot(path1.time, var(:,3), 'DisplayName', 'z MC', "color", "#0072BD")

%% display path vs estimation

sleep_duration = 0.1;
figure
hold on 
grid minor
view([-37.5 30]);
xlim([min(path1.path(:,1))-80 max(path1.path(:,1))+80]);
ylim([min(path1.path(:,2))-80 max(path1.path(:,2))+80]);
zlim([min(path1.path(:,3))-80 max(path1.path(:,3))+80]);
xlabel('x'); ylabel('y'); zlabel('z');

for i = 1:size(sensor_list, 2)
    sensor = sensor_list(i);
    color = color_list(i);
    sensor.calculate_measurements(path1.path);
    scatter3(sensor.sensor_position(:,1), sensor.sensor_position(:,2), sensor.sensor_position(:,3), 'filled', color);
end

% plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');
% plot3(estimated_path(:,1), estimated_path(:,2), estimated_path(:,3), 'r*');

ax1 = plot3(path1.path(1:1,1), path1.path(1:1,2), path1.path(1:1,3), 'b.-');
for i = 1:length(path1.path)
    delete(ax1);

    ax1 = plot3(path1.path(1:i,1), path1.path(1:i,2), path1.path(1:i,3), 'b.-');
    
    prediction_position = estimated_path(i,:);
    ax3 = plot3(prediction_position(1), prediction_position(2), prediction_position(3), 'r*'); 
    pause(sleep_duration);
    delete(ax3);
    ax3 = plot3(prediction_position(1), prediction_position(2), prediction_position(3), 'r.'); 
end

