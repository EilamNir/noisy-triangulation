clear; close all; clc;
%% create data
load('train_path.mat')
% XTrain = [];
% YTrain = [];
% for i = 3:length(path)-1
%     XTrain = cat(4,XTrain, path(i-2:i,:));
%     YTrain = cat(1,YTrain, path(i+1,:));
% end
% ix = randperm(length(YTrain));
% 
% XTrain = XTrain(:,:,:,ix);
% YTrain = YTrain(ix,:);

XTrain = noisy_distances;
YTrain = path;
ix = randperm(length(YTrain));

XTrain = XTrain(:,:,:,ix);
YTrain = YTrain(ix,:);

load('train_path_test.mat')

XVal = noisy_distances;
YVal = path;

ix = randperm(length(YVal));

XVal = XVal(:,:,:,ix);
YVal = YVal(ix,:);

%% 
layers = [ ...
    imageInputLayer([4 3 1])
    
    convolution2dLayer(3,8, 'Padding','same')
    reluLayer
    
    convolution2dLayer(3,16, 'Padding','same')
    reluLayer
    
    convolution2dLayer(3,32,'Padding','same')
    reluLayer   
    
    convolution2dLayer(3,32,'Padding','same')
    reluLayer 
    
    fullyConnectedLayer(10)
    reluLayer
    
    fullyConnectedLayer(3)
    regressionLayer];

maxEpochs = 7;
miniBatchSize = 32;

options = trainingOptions('adam', ...
    'LearnRateSchedule',"piecewise", ...
    'LearnRateDropFactor',0.1, ...
    'LearnRateDropPeriod',3, ...    
    'ExecutionEnvironment','cpu', ...
    'MaxEpochs',maxEpochs, ...
    'shuffle', 'every-epoch',...
    'InitialLearnRate',1e-2, ...
    'MiniBatchSize',miniBatchSize, ...
    'GradientThreshold',1, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'ValidationData', {XVal,YVal});

net = trainNetwork(XTrain,YTrain,layers,options);

%%
YPred = predict(net,XTrain(:,:,1,1:3));

%%
load('train_path_test.mat')
%load('train_path.mat')

XTest = noisy_distances;
YTest = path;


YPred = predict(net,XTest);

figure()
plot3(YTest(:,1), YTest(:,2), YTest(:,3), 'k.')
hold on
plot3(YPred(:,1), YPred(:,2), YPred(:,3), 'b.' )