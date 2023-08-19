clear; close all; clc;
%% simulation settings ****************************************************
noisy_distances = [];
path = [];
for i = 1:300   
    %SensorPos = [-5000,0,0; 400, -7400, 0; 800, 800, 0];
    SensorPos = randi([-5000, 5000],3,3);
    %TargetPos = [0,0,5000];
    TargetPos = randi([-5000, 5000], 1, 3);
    TargetSpeed_xy = 50;
    TargetSpeed_z = 10;
    TargetRotSpeed = 3;
    TimeRes = 0.5;
    %% 
    import generate_path
    path1 = generate_path("initial_phi", deg2rad(randi([0 360])), "initial_speed_xy", TargetSpeed_xy, "initial_speed_z", TargetSpeed_z, "initial_position", TargetPos, "TimeRes", TimeRes);
    path1.add_straight_interval(randi(100));
    samples = [-1 1];
    choose = samples(randi(numel(samples)));
    path1.add_xy_turn_interval(randi(100), -choose*deg2rad(TargetRotSpeed));
    path1.add_straight_interval(randi(100));
    
    for step = path1.path'
        perfect_distances = vecnorm(step - SensorPos,2,2);
        tmp = cat(1, SensorPos, (perfect_distances + random('Normal', 0, 15, size(perfect_distances)))');
        noisy_distances = cat(4, noisy_distances, tmp);
        path = cat(1, path, step');
    end
end
save('train_path.mat', 'path', 'noisy_distances');
