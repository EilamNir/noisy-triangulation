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
load('train_path_const_sen.mat')
XTrain = cat(4, XTrain, noisy_distances);
YTrain = cat(1, YTrain, path);
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
    
    convolution2dLayer(3,64, 'Padding','same')
    reluLayer
    
    convolution2dLayer(3,64, 'Padding','same')
    reluLayer
    
    convolution2dLayer(3,64,'Padding','same')
    reluLayer   
    
    convolution2dLayer(3,64,'Padding','same')
    reluLayer
    
    dropoutLayer(0.3)
    
    fullyConnectedLayer(1)
    regressionLayer];

maxEpochs = 20;
miniBatchSize = 256;
GradientThreshold = 1;
InitialLearnRate = 1e-2;
LearnRateDropPeriod = 5;

optionsx = trainingOptions('adam', ...
    'LearnRateSchedule',"piecewise", ...
    'LearnRateDropFactor',0.1, ...
    'LearnRateDropPeriod',LearnRateDropPeriod, ...    
    'ExecutionEnvironment','cpu', ...
    'MaxEpochs',maxEpochs, ...
    'shuffle', 'every-epoch',...
    'InitialLearnRate',InitialLearnRate, ...
    'MiniBatchSize',miniBatchSize, ...
    'GradientThreshold',GradientThreshold, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'ValidationData', {XVal,YVal(:,1)});

optionsy = trainingOptions('adam', ...
    'LearnRateSchedule',"piecewise", ...
    'LearnRateDropFactor',0.1, ...
    'LearnRateDropPeriod',LearnRateDropPeriod, ...    
    'ExecutionEnvironment','cpu', ...
    'MaxEpochs',maxEpochs, ...
    'shuffle', 'every-epoch',...
    'InitialLearnRate',InitialLearnRate, ...
    'MiniBatchSize',miniBatchSize, ...
    'GradientThreshold',GradientThreshold, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'ValidationData', {XVal,YVal(:,2)});

optionsz = trainingOptions('adam', ...
    'LearnRateSchedule',"piecewise", ...
    'LearnRateDropFactor',0.1, ...
    'LearnRateDropPeriod',LearnRateDropPeriod, ...    
    'ExecutionEnvironment','cpu', ...
    'MaxEpochs',maxEpochs, ...
    'shuffle', 'every-epoch',...
    'InitialLearnRate',InitialLearnRate, ...
    'MiniBatchSize',miniBatchSize, ...
    'GradientThreshold',GradientThreshold, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'ValidationData', {XVal,YVal(:,3)});
%net = trainNetwork(XTrain,YTrain,layers,options);


netx = trainNetwork(XTrain,YTrain(:,1),layers,optionsx);
nety = trainNetwork(XTrain,YTrain(:,2),layers,optionsy);
netz = trainNetwork(XTrain,YTrain(:,3),layers,optionsz);


%%
%YPred = predict(net,XTrain(:,:,1,1:3));

%%
load('train_path_test.mat')
%load('train_path.mat')

XTest = noisy_distances;
YTest = path;


YPredx = predict(netx,XTest);
YPredy = predict(nety,XTest);
YPredz = predict(netz,XTest);

YPred = [YPredx YPredy YPredz];

figure()
plot3(YTest(:,1), YTest(:,2), YTest(:,3), 'k.')
hold on
plot3(YPred(:,1), YPred(:,2), YPred(:,3), 'b.' )