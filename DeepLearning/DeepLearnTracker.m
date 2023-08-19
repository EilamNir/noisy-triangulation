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

maxEpochs = 20;
miniBatchSize = 32;

options = trainingOptions('adam', ...
    'LearnRateSchedule',"piecewise", ...
    'LearnRateDropFactor',0.1, ...
    'LearnRateDropPeriod',3, ...    
    'ExecutionEnvironment','cpu', ...
    'MaxEpochs',maxEpochs, ...
    'InitialLearnRate',1e-2, ...
    'MiniBatchSize',miniBatchSize, ...
    'GradientThreshold',1, ...
    'Verbose',false, ...
    'Plots','training-progress');

net = trainNetwork(XTrain,YTrain,layers,options);

%%
YPred = predict(net,XTrain(:,:,1,1:3));