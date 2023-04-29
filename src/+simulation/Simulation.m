close all; clear; clc;
%% simulation settings ****************************************************
%SensorPos = [-300,0,0;-100 -100 -100];
SensorPos = [-5000,0,0; 400, -7400, 0; 800, 800, 0];
TargetPos = [0,0,0];
TargetSpeed = 10;
TargetRotSpeed = 5;
TimeRes = 0.5;
SimulationDuration = 150;
TargetTheta = 90;
TargetPhi = 45;

% make the results reproducible by seeding the random number generator
rng(42);

%% generate a path
import simulation.generate_path

%% test straight lines and xy turns
path1 = generate_path("initial_phi", 0, "initial_speed_xy", 50, "initial_speed_z", 10, "initial_position", [0,0,1000], "TimeRes", 0.5);
path1.add_straight_interval(10);
path1.add_xy_turn_interval(10, -pi*3/(2 * 10));
path1.add_straight_interval(10);


figure
hold on
grid minor
view([-37.5 30]);
xlabel('x'); ylabel('y'); zlabel('z');
plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');



% %% test 3d turns
% for i = [0, 45, 90]
%     path2 = generate_path();
%     path2.add_straight_interval(20, "theta", 90, "phi", 0);
%     path2.add_3d_turn_interval(20, 10, i)
%     path2.add_straight_interval(30);

%     figure
%     hold on
%     grid minor
%     view([-37.5 30]);
%     xlabel('x'); ylabel('y'); zlabel('z');
%     plot3(path2.path(:,1), path2.path(:,2), path2.path(:,3), 'b.-');
% end

% % test sensors
import simulation.noisy_sensor

sensor_distance1_pos = [-5000,0,0];
sensor_distance1 = noisy_sensor(sensor_distance1_pos, "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);
sensor_distance2_pos = [400, -7400, 0];
sensor_distance2 = noisy_sensor(sensor_distance2_pos, "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);
sensor_distance3_pos = [800, 800, 0];
sensor_distance3 = noisy_sensor(sensor_distance3_pos, "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);
sensor_distance4_pos = [-500, -500, 500];
sensor_distance4 = noisy_sensor(sensor_distance4_pos, "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);

sensor_angle1_pos = [-5000,0,0];
sensor_angle1 = noisy_sensor(sensor_angle1_pos, "has_distance", false, "has_angle", true, "theta_noise_sigma", 5, "theta_noise_sigma", 5);
sensor_angle2_pos = [400, -7400, 0];
sensor_angle2 = noisy_sensor(sensor_angle2_pos, "has_distance", false, "has_angle", true, "theta_noise_sigma", 5, "theta_noise_sigma", 5);
sensor_angle3_pos = [800, 800, 0];
sensor_angle3 = noisy_sensor(sensor_angle3_pos, "has_distance", false, "has_angle", true, "theta_noise_sigma", 5, "theta_noise_sigma", 5);

sensor_list = [sensor_distance1, sensor_distance2, sensor_distance3];
% sensor_list = [sensor_angle1, sensor_angle2, sensor_angle3];
color_list = ['r', 'g', 'y', 'k'];

% figure
% hold on 
% grid minor
% view([-37.5 30]);
% xlim([min(path1.path(:,1))-10 max(path1.path(:,1))+10]);
% ylim([min(path1.path(:,2))-10 max(path1.path(:,2))+10]);
% zlim([min(path1.path(:,3))-10 max(path1.path(:,3))+10]);
% xlabel('x'); ylabel('y'); zlabel('z');
% plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');

for i = 1:size(sensor_list, 2)
    sensor = sensor_list(i);
    color = color_list(i);
    sensor.calculate_measurements(path1.path);
end

%%
% test iterative estimator
import estimation.iterative_estimator;

it = iterative_estimator(sensor_list, path1.path(1,:));

estimated_path = it.estimate_path_by_distance();

%%
% display path vs estimation
figure
hold on 
grid minor
view([-37.5 30]);
%xlim([min(path1.path(:,1))-10 max(path1.path(:,1))+10]);
%ylim([min(path1.path(:,2))-10 max(path1.path(:,2))+10]);
%zlim([min(path1.path(:,3))-10 max(path1.path(:,3))+10]);
xlabel('x'); ylabel('y'); zlabel('z');

for i = 1:size(sensor_list, 2)
    sensor = sensor_list(i);
    color = color_list(i);
    sensor.calculate_measurements(path1.path);
    scatter3(sensor.sensor_position(:,1), sensor.sensor_position(:,2), sensor.sensor_position(:,3), 'filled', color);
end
plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');

plot3(estimated_path(:,1), estimated_path(:,2), estimated_path(:,3), 'r*');

%%
% test estimator with working and non-working examples

% temp_path = [
%     1.0,1.0,0;
%     0.9,0.9,0;
%     0.8,0.8,0;
%     0.7,0.7,0;
%     0.6,0.6,0;
%     0.5,0.5,0;
%     0.4,0.4,0;
%     0.3,0.3,0;
%     0.2,0.2,0;
%     0.1,0.1,0;
%     0,0,0;
%     ];
% % working
% sensor_pos = [0,0,1; 1,0,0; 0,1,0; 0,0.8,0];
% get_path_from_sensor_pos(sensor_pos, temp_path);
% % figure()
% % plot3(sensor_pos(:,1),sensor_pos(:,2),sensor_pos(:,3));
% % hold on;
% % plot3(0.5,0.5,0, "r.");
% % non working
% temp_path = [
%     0.5,0.5,0.5;
%     0.5,0.5,0;
%     0.5,0.5,-0.5;
%     ];
% %sensor_pos = [0,0,0; 1,0,0; 0,1,0];
% %get_path_from_sensor_pos(sensor_pos, temp_path);

%% functions

% function estimated_path = get_path_from_sensor_pos(sensor_pos, path)
%     import simulation.noisy_sensor;
%     import estimation.iterative_estimator;
%     sensor_list = noisy_sensor.empty();
%     for i = 1:size(sensor_pos, 1)
%         sensor = noisy_sensor(sensor_pos(i,:), "has_distance", true, "has_angle", false, "distance_noise_sigma", 0.1);
%         sensor_list(i) = sensor;
%         sensor.calculate_measurements(path);
%     end

%     it = iterative_estimator(sensor_list, path(1,:));
%     estimated_path = it.estimate_path_by_distance();
%     display(estimated_path);
% end