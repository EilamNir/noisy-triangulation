clear; close all; clc;
%% create data
load('train_path_state.mat')
XTrain = noisy_distances;
YTrain = state_key';
% load('train_path_const_sen.mat')
% XTrain = cat(4, XTrain, noisy_distances);
% YTrain = cat(1, YTrain, path);
ix = randperm(length(YTrain));

XTrain = XTrain(:,:,:,ix);
YTrain = YTrain(ix,:);

load('train_path_state_test.mat')

XVal = noisy_distances;
YVal = state_key';

ix = randperm(length(YVal));

XVal = XVal(:,:,:,ix);
YVal = YVal(ix,:);

%% 
layers = [ ...
    imageInputLayer([4 3 3])
    
    convolution2dLayer(3,64, 'Padding','same')
    reluLayer
    convolution2dLayer(3,64, 'Padding','same')
    reluLayer 
    convolution2dLayer(3,64, 'Padding','same')
    reluLayer
    fullyConnectedLayer(2)
    softmaxLayer
    classificationLayer];

maxEpochs = 20;
miniBatchSize = 1024;
GradientThreshold = 2;
InitialLearnRate = 1e-3;
LearnRateDropPeriod = 5;

options = trainingOptions('adam', ...
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
    'ValidationData', {XVal,categorical(YVal)});
% 
% optionsy = trainingOptions('adam', ...
%     'LearnRateSchedule',"piecewise", ...
%     'LearnRateDropFactor',0.1, ...
%     'LearnRateDropPeriod',LearnRateDropPeriod, ...    
%     'ExecutionEnvironment','cpu', ...
%     'MaxEpochs',maxEpochs, ...
%     'shuffle', 'every-epoch',...
%     'InitialLearnRate',InitialLearnRate, ...
%     'MiniBatchSize',miniBatchSize, ...
%     'GradientThreshold',GradientThreshold, ...
%     'Verbose',false, ...
%     'Plots','training-progress',...
%     'ValidationData', {XVal,YVal(:,2)});
% 
% optionsz = trainingOptions('adam', ...
%     'LearnRateSchedule',"piecewise", ...
%     'LearnRateDropFactor',0.1, ...
%     'LearnRateDropPeriod',LearnRateDropPeriod, ...    
%     'ExecutionEnvironment','cpu', ...
%     'MaxEpochs',maxEpochs, ...
%     'shuffle', 'every-epoch',...
%     'InitialLearnRate',InitialLearnRate, ...
%     'MiniBatchSize',miniBatchSize, ...
%     'GradientThreshold',GradientThreshold, ...
%     'Verbose',false, ...
%     'Plots','training-progress',...
%     'ValidationData', {XVal,YVal(:,3)});
min_net = 10000;
for k = 1:100
    net = trainNetwork(XTrain,categorical(YTrain),layers,options);
    load('train_path_state_test.mat')
    %load('train_path.mat')

    XTest = noisy_distances;
    YTest = state_key;


    YPred = predict(net,XTest);
    if sum(YPred(:,2)'-YTest) < min_net
        min_net = sum(YPred(:,2)'-YTest);
        save('net', 'net')
    end
end

% netx = trainNetwork(XTrain,YTrain(:,1),layers,optionsx);
% nety = trainNetwork(XTrain,YTrain(:,2),layers,optionsy);
% netz = trainNetwork(XTrain,YTrain(:,3),layers,optionsz);


%%
%YPred = predict(net,XTrain(:,:,1,1:3));

%%
load('train_path_state_test.mat')
%load('train_path.mat')

XTest = noisy_distances;
YTest = state_key;


YPred = predict(net,XTest);
% YPredy = predict(nety,XTest);
% YPredz = predict(netz,XTest);
% 
% YPred = [YPredx YPredy YPredz];

figure()
plot(YTest, 'k*')
hold on
plot(YPred(:,1), 'b.' )
plot(YPred(:,2), 'r.' )