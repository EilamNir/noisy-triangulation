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


%% noise ******************************************************************
DistanceNoiseForm = 'Normal';
DistanceNoiseMu = 0;
DistanceNoiseSigma = 10;

AngleNoiseForm = 'Normal';
AngleNoiseMu = 0;
AngleNoiseSigma = 0;

% make the results reproducible by seeding the random number generator
rng(42);

% %% main loop **************************************************************
% t = 0:TimeRes:SimulationDuration;
% figure
% scatter3(SensorPos(:,1), SensorPos(:,2), SensorPos(:,3), 'filled', 'r');
% axis([-500 500 -500 500 -500 500])
% xlabel('x'); ylabel('y'); zlabel('z');
% hold on
% grid minor
% for i = t
%     % if exist('ax1','var')                                                  % remove old plot
%     %     pause(0.01)
%     %     % delete([ax1,ax2,ax3]);
%     % end
%     ax1 = plot3(TargetPos(:,1), TargetPos(:,2), TargetPos(:,3), 'b');
%     ax2 = scatter3(TargetPos(end,1), TargetPos(end,2), TargetPos(end,3), 'filled', 'b');
    
%     TargetPos(end+1,:) = TargetPos(end,:) + (TimeRes*TargetSpeed.*TransposeMatrix(TargetPhi, TargetTheta))';
    
%     % Target execute loop 
%     if i > 40 && i < 40+54.5
%        TargetTheta = TargetTheta - TimeRes*TargetRotSpeed;
%     end
    
%     % sensor prediction 
%     PredictDistance = vecnorm(TargetPos(end,:)-SensorPos,2,2) + random(DistanceNoiseForm, DistanceNoiseMu, DistanceNoiseSigma);
%     PredictAnglePhi = atan2d(TargetPos(end,2)-SensorPos(:,2),TargetPos(end,1)-SensorPos(:,1)) + random(AngleNoiseForm, AngleNoiseMu, AngleNoiseSigma);
%     PredictAngleTheta = atan2d(norm(TargetPos(end,1:2)-SensorPos(:,1:2)),TargetPos(end,3)-SensorPos(:,3)) + random(AngleNoiseForm, AngleNoiseMu, AngleNoiseSigma);
%     PredictTargetPos = SensorPos + PredictDistance.*reshape(TransposeMatrix(PredictAnglePhi, PredictAngleTheta),[],3);
    
%     ax3 = plot3(PredictTargetPos(:,1), PredictTargetPos(:,2), PredictTargetPos(:,3), '*k'); 
% end

% generate a path
import simulation.generate_path

path1 = generate_path();
path1.TimeRes = 0.1;
path1.add_straight_interval(20, 90, 45);
path1.add_straight_interval(10, 80, 45);
path1.add_xy_turn_interval(10, 5, false);
path1.add_xy_turn_interval(10, -5);
path1.add_xy_turn_interval(20, 15, true, 100);
path1.add_straight_interval(10, 90, 90);

path_data = path1.path();

figure
scatter3(SensorPos(:,1), SensorPos(:,2), SensorPos(:,3), 'filled', 'r');
% axis([-500 500 -500 500 -500 500]);
hold on
grid minor
xlabel('x'); ylabel('y'); zlabel('z');
plot3(path_data(:,1), path_data(:,2), path_data(:,3), 'b.-');


for i = [0, 45, 90]
    path2 = generate_path();
    path2.add_straight_interval(20, 90, 45);
    path2.add_3d_turn_interval(20, 10, i)
    path2.add_straight_interval(30);

    path_data = path2.path();

    figure
    scatter3(SensorPos(:,1), SensorPos(:,2), SensorPos(:,3), 'filled', 'r');
    % axis([-500 500 -500 500 -500 500]);
    hold on
    grid minor
    xlabel('x'); ylabel('y'); zlabel('z');
    plot3(path_data(:,1), path_data(:,2), path_data(:,3), 'b.-');
end
