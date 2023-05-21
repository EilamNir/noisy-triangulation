close all; clear; clc;
%% simulation settings ****************************************************
SensorPos = [-5000,0,0; 400, -7400, 0; 800, 800, 0];
color_list = ['r', 'g', 'y', 'k', 'm'];
TargetPos = [0,0,1000];
TargetSpeed_xy = 50;
TargetSpeed_z = 10;
TargetRotSpeed = 3;
TimeRes = 0.5;

% make the results reproducible by seeding the random number generator
rng(42);

should_generate_data = false;

if should_generate_data
    % generate a path
    import simulation.generate_path
    path1 = generate_path("initial_phi", 0, "initial_speed_xy", TargetSpeed_xy, "initial_speed_z", TargetSpeed_z, "initial_position", TargetPos, "TimeRes", TimeRes);
    path1.add_straight_interval(100);
    path1.add_xy_turn_interval(90, -deg2rad(TargetRotSpeed));
    path1.add_straight_interval(100);

    % create estimation and save to file
    import simulation.create_simulations.save_simulation_MC
    % [true_path, path_time, estimated_path, MC_MSE, cov_mat, cov_MSE, SensorPos] = save_simulation_MC(SensorPos, path1.path, path1.time, 10000, "../data/MC_original.mat");
    save_simulation_MC(SensorPos, path1.path, path1.time, 10000, "../data/MC_original_10000.mat");
    save_simulation_MC([-1000,-4000,0; -2000,-6000,0; 0,-5000,0], path1.path, path1.time, 10000, "../data/MC_bunched_sensors_10000.mat");
    save_simulation_MC([-5000,0,5000; 400, -7400, 5000; 800, 800, 5000], path1.path, path1.time, 10000, "../data/MC_high_sensors_10000.mat");
    save_simulation_MC([-5000,0,0; 400, -7400, 0; 800, 800, 0; 8000, 1000, 0], path1.path, path1.time, 10000, "../data/MC_4_sensors_10000.mat");
else
    % load estimation from file
    import simulation.create_simulations.load_simulation_MC
    filenames = [ ...
        "../data/MC_original_10000.mat", ...
        "../data/MC_bunched_sensors_10000.mat", ...
        "../data/MC_high_sensors_10000.mat", ...
        "../data/MC_4_sensors_10000.mat"...
        ];
    for file_index = 1:size(filenames, 2)
        filename = filenames(file_index);
        [true_path, path_time, estimated_path, MC_MSE, cov_mat, cov_MSE, SensorPos] = load_simulation_MC(filename);

        GDOP = sqrt(sum(cov_mat, 2));
        HDOP = sqrt(sum(cov_mat(:, 1:2), 2));
        VDOP = sqrt(cov_mat(:, 3));
        figure
        title(filename, 'Interpreter', 'none')
        hold on
        grid minor
        xlabel('time');
        ylabel('error');
        legend('Location','east');
        plot(path_time, GDOP, 'DisplayName', 'GDOP')
        plot(path_time, VDOP, 'DisplayName', 'VDOP')
        plot(path_time, HDOP, 'DisplayName', 'HDOP')

        figure
        title(filename, 'Interpreter', 'none')
        hold on
        grid minor
        xlabel('time');
        ylabel('error');
        legend('Location','east');
        plot(path_time, cov_MSE(:,1), '--','DisplayName', 'x cov', "color", "r")
        plot(path_time, MC_MSE(:,1), 'DisplayName', 'x MC', "color", "#A2142F")
        plot(path_time, cov_MSE(:,2), '--','DisplayName', 'y cov', "color", "g")
        plot(path_time, MC_MSE(:,2), 'DisplayName', 'y MC', "color", "#77AC30")
        plot(path_time, cov_MSE(:,3), '--','DisplayName', 'z cov', "color", "b")
        plot(path_time, MC_MSE(:,3), 'DisplayName', 'z MC', "color", "#0072BD")

        %% display path vs estimation

        sleep_duration = 0.1;
        figure
        title(filename, 'Interpreter', 'none')
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


        ax1 = plot3(true_path(1:1,1), true_path(1:1,2), true_path(1:1,3), 'b.-');
        for i = 1:length(true_path)
            delete(ax1);

            ax1 = plot3(true_path(1:i,1), true_path(1:i,2), true_path(1:i,3), 'b.-');
            
            prediction_position = estimated_path(i,:);
            ax3 = plot3(prediction_position(1), prediction_position(2), prediction_position(3), 'r*'); 
            % pause(sleep_duration);
            delete(ax3);
            ax3 = plot3(prediction_position(1), prediction_position(2), prediction_position(3), 'r.'); 
        end
    end
end
