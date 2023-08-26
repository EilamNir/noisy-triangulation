clear; close all; clc;
%% simulation settings ****************************************************
noisy_distances = [];
path = [];
state_key = [];
f = waitbar(0,'Please wait...');
data_number = 1000;
for i = 1:data_number  
    %SensorPos = [-4000,0,0; 400, -3800, 0; 800, 800, 0];
    SensorPos = randi([-7000, 7000],3,3);
%    SensorPos = [-4000,0,0; 400, -3800, 0; 800, 800, 0];
    %TargetPos = [0,0,5000];
    TargetPos = randi([-7000, 7000], 1, 3);
    TargetSpeed_xy = 50;
    TargetSpeed_z = 10;
    TargetRotSpeed = 3;
    TimeRes = 0.5;
    %% 
    import generate_path.*
    path1 = generate_path("initial_phi", deg2rad(randi([0 360])), "initial_speed_xy", TargetSpeed_xy, "initial_speed_z", TargetSpeed_z, "initial_position", TargetPos, "TimeRes", TimeRes);
    path1.add_straight_interval(randi(50));
    samples = [-1 1];
    choose = samples(randi(numel(samples)));
    path1.add_xy_turn_interval(randi(50), -choose*deg2rad(TargetRotSpeed));
    
    figure 
    plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3))
    axis equal
    close all;
    samples = 3;
    
    for j = 1:length(path1.path')-samples
        tmp = [];
        noisy_distances_tmp = [];
        for k = 0:samples-1
            perfect_distances = vecnorm(path1.path(j+k,:)' - SensorPos,2,2);
            tmp = cat(1, SensorPos, (perfect_distances + random('Normal', 0, 20, size(perfect_distances)))');
            noisy_distances_tmp = cat(3, noisy_distances_tmp, tmp);
        end
        noisy_distances = cat(4, noisy_distances, noisy_distances_tmp);
        path = cat(1, path, path1.path(j,:));
    end
    state_key = cat(2, state_key, path1.state_key(samples+1:end));
    waitbar(i/data_number,f,'Create your data');
end
close(f)
save('train_path_state.mat', 'state_key', 'noisy_distances');
%% 
figure
plot3(path(:,1), path(:,2), path(:,3))