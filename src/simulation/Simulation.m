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

%% help function **********************************************************
TransposeMatrix = @(Phi, Theta) [cosd(Phi).*sind(Theta); sind(Phi).*sind(Theta); cosd(Theta)];

%% main loop **************************************************************
t = 0:TimeRes:SimulationDuration;
figure
scatter3(SensorPos(:,1), SensorPos(:,2), SensorPos(:,3), 'filled', 'r');
axis([-500 500 -500 500 -500 500])
xlabel('x'); ylabel('y'); zlabel('z');
hold on
grid minor
for i = t
    if exist('ax1','var')                                                  % remove old plot
        pause(0.1)
        delete([ax1,ax2,ax3]);
    end
    ax1 = plot3(TargetPos(:,1), TargetPos(:,2), TargetPos(:,3), 'b');
    ax2 = scatter3(TargetPos(end,1), TargetPos(end,2), TargetPos(end,3), 'filled', 'b');
    
    TargetPos(end+1,:) = TargetPos(end,:) + (TimeRes*TargetSpeed.*TransposeMatrix(TargetPhi, TargetTheta))';
    
    % Target execute loop 
    if i > 40 && i < 40+54.5
       TargetTheta = TargetTheta - TimeRes*TargetRotSpeed;
    end
    
    % sensor prediction 
    PredictDistance = vecnorm(TargetPos(end,:)-SensorPos,2,2) + random(DistanceNoiseForm, DistanceNoiseMu, DistanceNoiseSigma);
    PredictAnglePhi = atan2d(TargetPos(end,2)-SensorPos(:,2),TargetPos(end,1)-SensorPos(:,1)) + random(AngleNoiseForm, AngleNoiseMu, AngleNoiseSigma);
    PredictAngleTheta = atan2d(norm(TargetPos(end,1:2)-SensorPos(:,1:2)),TargetPos(end,3)-SensorPos(:,3)) + random(AngleNoiseForm, AngleNoiseMu, AngleNoiseSigma);
    PredictTargetPos = SensorPos + PredictDistance.*reshape(TransposeMatrix(PredictAnglePhi, PredictAngleTheta),[],3);
    
    ax3 = plot3(PredictTargetPos(:,1), PredictTargetPos(:,2), PredictTargetPos(:,3), '*k'); 
end
