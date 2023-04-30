close all; clear; clc;
%% simulation settings ****************************************************
% SensorPos = [-300,0,0;1700 -1100 10];
% SensorPos = [-5000,0,0;1700 -7000 0;2000 1000 0];
SensorPos = [-5000,0,0;400 -7400 0;800 800 0];
% SensorPos = [-300,0,0;0 -100 0];
% SensorPos = [-300,0,0];
TargetPos = [0,0,0];
TargetSpeed = 50;
TargetRotSpeed = 10;
TimeRes = 0.5;
SimulationDuration = 150;
TargetTheta = 90;
TargetPhi = 0;


% make the results reproducible by seeding the random number generator
rng(42);

%% generate a path
import simulation.generate_path

%% test straight lines and xy turns
path1 = generate_path("initial_phi", 0, "initial_speed_xy", 50, "initial_speed_z", 10, "initial_position", [0,0,0], "TimeRes", 0.1);
path1.add_straight_interval(10);
path1.add_xy_turn_interval(10, -pi*3/(2 * 10));
path1.add_straight_interval(10);

true_path = path1.path;
targetPos = true_path(1);

%% noise ******************************************************************
DistanceNoiseForm = 'Normal';
DistanceNoiseMu = 0;
DistanceNoiseSigma = 10;

AngleNoiseForm = 'Normal';
AngleNoiseMu = 0;
AngleNoiseSigma = 0;

%% help function **********************************************************
% TransposeMatrix = @(Phi, Theta) [cosd(Phi).*sind(Theta); sind(Phi).*sind(Theta); cosd(Theta)];
TransposeMatrix = @(Phi) [cosd(Phi); sind(Phi)];
H = @(x) (x)./vecnorm(x,2,2);
%% main loop **************************************************************
% t = 0:TimeRes:SimulationDuration;
figure
scatter3(SensorPos(:,1), SensorPos(:,2), SensorPos(:,3), 'filled', 'r');
xlabel('x'); ylabel('y'); zlabel('z');
hold on
grid minor

PredictTargetPos = TargetPos;
for i = 1:(size(true_path, 1) - 1)
    if exist('ax1','var')                                                  % remove old plot
        pause(0.1)
        delete([ax1, ax2, ax4, ax5, ax6]);
    end
    ax1 = plot3(TargetPos(:,1), TargetPos(:,2), TargetPos(:,3), 'b');
    ax2 = scatter3(TargetPos(end,1), TargetPos(end,2), TargetPos(end,3), 'filled', 'b');
    
    TargetPos(end+1,:) = true_path(i + 1, :);
    % TargetPos(end+1,1:2) = TargetPos(end,1:2) + (TimeRes*TargetSpeed.*TransposeMatrix(TargetPhi))';
    % TargetPos(end,3) = TargetPos(end-1,3) + 10*TimeRes;
    
    % % Target execute loop 
    % if i > 40 && i < 40+27.5
    %    TargetPhi = TargetPhi + TimeRes*(TargetRotSpeed);
    % end
    
    % sensor prediction 
    PredictDistance = vecnorm(TargetPos(end,:)-SensorPos,2,2) + random(DistanceNoiseForm, DistanceNoiseMu, DistanceNoiseSigma);
%     PredictAnglePhi = atan2d(TargetPos(end,2)-SensorPos(:,2),TargetPos(end,1)-SensorPos(:,1)) + random(AngleNoiseForm, AngleNoiseMu, AngleNoiseSigma);
%     PredictAngleTheta = atan2d(norm(TargetPos(end,1:2)-SensorPos(:,1:2)),TargetPos(end,3)-SensorPos(:,3)) + random(AngleNoiseForm, AngleNoiseMu, AngleNoiseSigma);
%     PredictTargetPos = SensorPos + PredictDistance.*reshape(TransposeMatrix(PredictAnglePhi, PredictAngleTheta),[],3);
%     while true
    for j = 1:10
        X0Distance = vecnorm(PredictTargetPos-SensorPos,2,2);
        if abs(PredictDistance - X0Distance) < 20
            break;
        end
        PredictTargetPos = PredictTargetPos + (inv(H(PredictTargetPos-SensorPos)+eye(3,3)*0.1)*(PredictDistance-X0Distance))';
        
%         disp(PredictTargetPos(:,1)./X0Distance + (abs(PredictTargetPos(:,1)./X0Distance)<0.5))
%         PredictTargetPos(:,1) = PredictTargetPos(:,1) + sign(PredictTargetPos(:,1)-SensorPos(:,1)).*((PredictDistance - X0Distance)./(PredictTargetPos(:,1)./X0Distance + (abs(PredictTargetPos(:,1)./X0Distance)<0.5)));
%         X0Distance = vecnorm(mean(PredictTargetPos)-SensorPos,2,2);
%         PredictTargetPos(:,2) = PredictTargetPos(:,2) + sign(PredictTargetPos(:,2)-SensorPos(:,2)).*((PredictDistance - X0Distance)./(PredictTargetPos(:,2)./X0Distance + (abs(PredictTargetPos(:,2)./X0Distance)<0.5)));
%         X0Distance = vecnorm(mean(PredictTargetPos)-SensorPos,2,2);
%         PredictTargetPos(:,3) = PredictTargetPos(:,3) + sign(PredictTargetPos(:,3)-SensorPos(:,3)).*((PredictDistance - X0Distance)./(PredictTargetPos(:,3)./X0Distance + (abs(PredictTargetPos(:,3)./X0Distance)<0.5)));
    end


    ax3 = plot3(mean(PredictTargetPos(:,1)), mean(PredictTargetPos(:,2)), mean(PredictTargetPos(:,3)), '*k'); 
    [X,Y,Z] = sphere;
    
    ax4 = surf(X*PredictDistance(1)+SensorPos(1,1),Y*PredictDistance(1)+SensorPos(1,2),Z*PredictDistance(1)+SensorPos(1,3),...
        'EdgeColor','none','FaceAlpha',0.2,'FaceColor','red');
    ax5 = surf(X*PredictDistance(2)+SensorPos(2,1),Y*PredictDistance(2)+SensorPos(2,2),Z*PredictDistance(2)+SensorPos(2,3),...
        'EdgeColor','none','FaceAlpha',0.2,'FaceColor','red');
    ax6 = surf(X*PredictDistance(3)+SensorPos(3,1),Y*PredictDistance(3)+SensorPos(3,2),Z*PredictDistance(3)+SensorPos(3,3),...
        'EdgeColor','none','FaceAlpha',0.2,'FaceColor','red');
    axis equal
end
