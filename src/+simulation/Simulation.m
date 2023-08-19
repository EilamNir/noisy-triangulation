close all; clear; clc;
%% simulation settings ****************************************************
SensorPos = [-5000,0,0; 400, -7400, 0; 800, 800, 0];
color_list = ['r', 'g', 'y', 'k', 'm'];
TargetPos = [0,0,5000];
TargetSpeed_xy = 50;
TargetSpeed_z = 10;
TargetRotSpeed = 3;
TimeRes = 0.5;
should_generate_data = false;
animation = true;
%%

% make the results reproducible by seeding the random number generator
rng(42);


if should_generate_data
    % generate a path
    import simulation.generate_path
    path1 = generate_path("initial_phi", 0, "initial_speed_xy", TargetSpeed_xy, "initial_speed_z", TargetSpeed_z, "initial_position", TargetPos, "TimeRes", TimeRes);
    path1.add_straight_interval(100);
    path1.add_xy_turn_interval(90, -deg2rad(TargetRotSpeed));
    path1.add_straight_interval(100);

    % create estimation and save to file
    import simulation.create_simulations.save_simulation_MC
    % save_simulation_MC(SensorPos, path1.path, path1.time, 10000, "../data/MC_original_10000.mat");
    % save_simulation_MC([-1000,-4000,0; -2000,-6000,0; 0,-5000,0], path1.path, path1.time, 10000, "../data/MC_bunched_sensors_10000.mat");
    % save_simulation_MC([-5000,0,-5000; 400, -7400, -5000; 800, 800, -5000], path1.path, path1.time, 10000, "../data/MC_low_sensors_10000.mat");
    % save_simulation_MC([-5000,0,-1000; 400, -7400, 0; 800, 800, 500; 8000, 1000, 0], path1.path, path1.time, 100, "../data/non_iter_navidi_100.mat");
%     save_simulation_MC([-5000,0,0; 400, -7400, 0; 800, 800, 0], path1.path, path1.time, 100, "../data/non_iter_navidi_3_sensors_100.mat");
    save_simulation_MC([-4000,0,0; 400, -3800, 0; 800, 800, 0], path1.path, path1.time, 100, "../data/deep_learn_sensors_100.mat");
else
    % load estimation from file
    [file,path] = uigetfile('../data/*.mat',...
                            'Select One or More Files', ...
                            'MultiSelect', 'on');
    if ~iscell(file)
        temp_cell1 = cell(1);
        temp_cell1{1} = file;
        file = temp_cell1;
        temp_cell2 = cell(1);
        temp_cell2{1} = path;
        path = temp_cell2;
    end
    filenames = fullfile(path,file);
    SimNum = size(filenames,2);  
    for file_index = 1:SimNum
        filename = filenames{file_index};
        load(filename);

        GDOP_iter = sqrt(sum(cov_mat_iter, 2));
        HDOP_iter = sqrt(sum(cov_mat_iter(:, 1:2), 2));
        VDOP_iter = sqrt(cov_mat_iter(:, 3));
        GDOP_non_iter = sqrt(sum(cov_mat_non_iter, 2));
        HDOP_non_iter = sqrt(sum(cov_mat_non_iter(:, 1:2), 2));
        VDOP_non_iter = sqrt(cov_mat_non_iter(:, 3));
        GDOP_deep_learn = sqrt(sum(cov_mat_deep_learn, 2));
        HDOP_deep_learn = sqrt(sum(cov_mat_deep_learn(:, 1:2), 2));
        VDOP_deep_learn = sqrt(cov_mat_deep_learn(:, 3));
        
        subplot(3,SimNum,file_index)
        title(file{file_index}, 'Interpreter', 'none')
        hold on
        grid minor
        xlabel('time');
        ylabel('error');
        legend();
        plot(path_time, GDOP_iter, 'DisplayName', 'GDOP iter')
        plot(path_time, VDOP_iter, 'DisplayName', 'VDOP iter')
        plot(path_time, HDOP_iter, 'DisplayName', 'HDOP iter')
%         plot(path_time, GDOP_non_iter, 'DisplayName', 'GDOP non iter')
%         plot(path_time, VDOP_non_iter, 'DisplayName', 'VDOP non iter')
%         plot(path_time, HDOP_non_iter, 'DisplayName', 'HDOP non iter')
        plot(path_time, GDOP_deep_learn, 'DisplayName', 'GDOP deep learn')
        plot(path_time, VDOP_deep_learn, 'DisplayName', 'VDOP deep learn')
        plot(path_time, HDOP_deep_learn, 'DisplayName', 'HDOP deep learn')


        subplot(3,SimNum,file_index+SimNum)
        title(file{file_index}, 'Interpreter', 'none')
        hold on
        grid minor
        xlabel('time');
        ylabel('error');
        legend();
        plot(path_time, (cov_MSE_iter(:,1)).^0.5, '--','DisplayName', 'x cov iter')
        plot(path_time, (MC_MSE_iter(:,1)).^0.5, 'DisplayName', 'x MC iter')
        plot(path_time, (cov_MSE_iter(:,2)).^0.5, '--','DisplayName', 'y cov iter')
        plot(path_time, (MC_MSE_iter(:,2)).^0.5, 'DisplayName', 'y MC iter')
        plot(path_time, (cov_MSE_iter(:,3)).^0.5, '--','DisplayName', 'z cov iter')
        plot(path_time, (MC_MSE_iter(:,3)).^0.5, 'DisplayName', 'z MC iter')
        

%         plot(path_time, (cov_MSE_non_iter(:,1)).^0.5, '--','DisplayName', 'x cov non iter')
%         plot(path_time, (MC_MSE_non_iter(:,1)).^0.5, 'DisplayName', 'x MC non iter')
%         plot(path_time, (cov_MSE_non_iter(:,2)).^0.5, '--','DisplayName', 'y cov non iter')
%         plot(path_time, (MC_MSE_non_iter(:,2)).^0.5, 'DisplayName', 'y MC non iter')
%         plot(path_time, (cov_MSE_non_iter(:,3)).^0.5, '--','DisplayName', 'z cov non iter')
%         plot(path_time, (MC_MSE_non_iter(:,3)).^0.5, 'DisplayName', 'z MC non iter')
        
        plot(path_time, (cov_MSE_deep_learn(:,1)).^0.5, '--','DisplayName', 'x cov deep lean')
        plot(path_time, (MC_MSE_deep_learn(:,1)).^0.5, 'DisplayName', 'x MC deep lean')
        plot(path_time, (cov_MSE_deep_learn(:,2)).^0.5, '--','DisplayName', 'y cov deep lean')
        plot(path_time, (MC_MSE_deep_learn(:,2)).^0.5, 'DisplayName', 'y MC deep lean')
        plot(path_time, (cov_MSE_deep_learn(:,3)).^0.5, '--','DisplayName', 'z cov deep lean')
        plot(path_time, (MC_MSE_deep_learn(:,3)).^0.5, 'DisplayName', 'z MC deep lean')

        %% display path vs estimation

        sleep_duration = 0.1;
        subplot(3,SimNum,file_index+2*SimNum)
        title(file{file_index}, 'Interpreter', 'none')
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


        
        if animation
            ax1 = plot3(true_path(1:1,1), true_path(1:1,2), true_path(1:1,3), 'b.-');
            for i = 1:length(true_path)
                delete(ax1);

                ax1 = plot3(true_path(1:i,1), true_path(1:i,2), true_path(1:i,3), 'b.-');

                prediction_position_iter = estimated_path_iter(i,:);
                prediction_position_non_iter = estimated_path_non_iter(i,:);
                prediction_position_deep_learn = estimated_path_deep_learn(i,:);
                ax3 = plot3(prediction_position_iter(1), prediction_position_iter(2), prediction_position_iter(3), 'r*'); 
                ax3 = plot3(prediction_position_non_iter(1), prediction_position_non_iter(2), prediction_position_non_iter(3), 'g*');
                ax3 = plot3(prediction_position_deep_learn(1), prediction_position_deep_learn(2), prediction_position_deep_learn(3), 'y*');
                pause(sleep_duration);
                delete(ax3);
                ax3 = plot3(prediction_position_iter(1), prediction_position_iter(2), prediction_position_iter(3), 'r.'); 
                ax3 = plot3(prediction_position_non_iter(1), prediction_position_non_iter(2), prediction_position_non_iter(3), 'g.');
                ax3 = plot3(prediction_position_deep_learn(1), prediction_position_deep_learn(2), prediction_position_deep_learn(3), 'y.'); 
            end
        else
            plot3(true_path(:,1), true_path(:,2), true_path(:,3), 'b.-');
            plot3(estimated_path_iter(:,1), estimated_path_iter(:,2), estimated_path_iter(:,3), 'r.');
            plot3(estimated_path_non_iter(:,1), estimated_path_non_iter(:,2), estimated_path_non_iter(:,3), 'g.');
            plot3(estimated_path_deep_learn(:,1), estimated_path_deep_learn(:,2), estimated_path_deep_learn(:,3), 'y.');
        end
    end
end
