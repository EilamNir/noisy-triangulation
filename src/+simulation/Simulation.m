close all; clear; clc;
%% simulation settings ****************************************************
%SensorPos = [-300,0,0;-100 -100 -100];
SensorPos = [-300,0,0];
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
path1 = generate_path();
% path1.TimeRes = 0.1;
path1.add_straight_interval(20, theta=90, phi=45);
path1.add_straight_interval(10, theta=80, phi=45);
path1.add_xy_turn_interval(10, 5, override_theta=false);
path1.add_xy_turn_interval(10, -5);
path1.add_xy_turn_interval(20, 15, override_theta=true, theta=100);
path1.add_straight_interval(10, theta=90, phi=90);

figure
hold on
grid minor
view([-37.5 30]);
xlabel('x'); ylabel('y'); zlabel('z');
plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');


%% test 3d turns
for i = [0, 45, 90]
    path2 = generate_path();
    path2.add_straight_interval(20, theta=90, phi=45);
    path2.add_3d_turn_interval(20, 10, i)
    path2.add_straight_interval(30);

    figure
    hold on
    grid minor
    view([-37.5 30]);
    xlabel('x'); ylabel('y'); zlabel('z');
    plot3(path2.path(:,1), path2.path(:,2), path2.path(:,3), 'b.-');
end

% test sensors
import simulation.noisy_sensor

sensor1 = noisy_sensor(SensorPos, has_distance=true, has_angle=true, distance_noise_sigma=50);
sensor2_pos = [1500,1600,150];
sensor2 = noisy_sensor(sensor2_pos, has_distance=true, has_angle=true, distance_noise_sigma=50);
sensor3_pos = [-310,10,0];
sensor3 = noisy_sensor(sensor3_pos, has_distance=true, has_angle=true, phi_noise_sigma=1, theta_noise_sigma=1);
sensor4_pos = [1510,1610,150];
sensor4 = noisy_sensor(sensor4_pos, has_distance=true, has_angle=true, phi_noise_sigma=1, theta_noise_sigma=1);

sensor_list = [sensor1, sensor2, sensor3, sensor4];
color_list = ['r', 'g', 'y', 'c'];

figure
hold on 
grid minor
view([-37.5 30]);
xlabel('x'); ylabel('y'); zlabel('z');
plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');

for i = 1:size(sensor_list, 2)
    sensor = sensor_list(i);
    color = color_list(i);
    sensor.calculate_measurements(path1.path);

    % convert sensor output back to estimated positions
    import utils.matrix_helpers.TransposeMatrix
    noisy_positions = sensor.sensor_position + sensor.noisy_distances .* (TransposeMatrix(sensor.noisy_phis, sensor.noisy_thetas)');
    perfect_positions = sensor.sensor_position + sensor.perfect_distances .* (TransposeMatrix(sensor.perfect_phis, sensor.perfect_thetas)');

    scatter3(sensor.sensor_position(:,1), sensor.sensor_position(:,2), sensor.sensor_position(:,3), 'filled', color);
    plot3(noisy_positions(:,1), noisy_positions(:,2), noisy_positions(:,3), ['*' color]);
    % plot3(perfect_positions(:,1), perfect_positions(:,2), perfect_positions(:,3), '^g');
end
