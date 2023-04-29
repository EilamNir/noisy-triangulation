close all; clear; clc;
%% simulation settings ****************************************************
%SensorPos = [-300,0,0;-100 -100 -100];
SensorPos = [-5000,0,0; 400, -7400, 0; 800, 800, 0];
TargetPos = [0,0,0];
TargetSpeed = 10;
TargetRotSpeed = 5;
TimeRes = 0.5;
SimulationDuration = 150;
TargetTheta = 90;
TargetPhi = 45;

% make the results reproducible by seeding the random number generator
rng(42);

%% generate a path
import simulation.generate_path

%% test straight lines and xy turns
path1 = generate_path("initial_phi", 0, "initial_speed_xy", 50, "initial_speed_z", 10, "initial_position", [0,0,1000], "TimeRes", 0.1);
path1.add_straight_interval(10);
path1.add_xy_turn_interval(10, -pi*3/(2 * 10));
path1.add_straight_interval(10);


figure
hold on
grid minor
view([-37.5 30]);
xlabel('x'); ylabel('y'); zlabel('z');
plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');



% %% test 3d turns
% for i = [0, 45, 90]
%     path2 = generate_path();
%     path2.add_straight_interval(20, "theta", 90, "phi", 0);
%     path2.add_3d_turn_interval(20, 10, i)
%     path2.add_straight_interval(30);

%     figure
%     hold on
%     grid minor
%     view([-37.5 30]);
%     xlabel('x'); ylabel('y'); zlabel('z');
%     plot3(path2.path(:,1), path2.path(:,2), path2.path(:,3), 'b.-');
% end

% % test sensors
import simulation.noisy_sensor

sensor_distance1_pos = [-5000,0,0];
sensor_distance1 = noisy_sensor(sensor_distance1_pos, "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);
sensor_distance2_pos = [400, -7400, 0];
sensor_distance2 = noisy_sensor(sensor_distance2_pos, "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);
sensor_distance3_pos = [800, 800, 0];
sensor_distance3 = noisy_sensor(sensor_distance3_pos, "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);
sensor_distance4_pos = [-500, -500, 500];
sensor_distance4 = noisy_sensor(sensor_distance4_pos, "has_distance", true, "has_angle", false, "distance_noise_sigma", 15);

sensor_angle1_pos = [-5000,0,0];
sensor_angle1 = noisy_sensor(sensor_angle1_pos, "has_distance", false, "has_angle", true, "theta_noise_sigma", 5, "theta_noise_sigma", 5);
sensor_angle2_pos = [400, -7400, 0];
sensor_angle2 = noisy_sensor(sensor_angle2_pos, "has_distance", false, "has_angle", true, "theta_noise_sigma", 5, "theta_noise_sigma", 5);
sensor_angle3_pos = [800, 800, 0];
sensor_angle3 = noisy_sensor(sensor_angle3_pos, "has_distance", false, "has_angle", true, "theta_noise_sigma", 5, "theta_noise_sigma", 5);

sensor_list = [sensor_distance1, sensor_distance2, sensor_distance3];
% sensor_list = [sensor_angle1, sensor_angle2, sensor_angle3];
color_list = ['r', 'g', 'y', 'k'];

% figure
% hold on 
% grid minor
% view([-37.5 30]);
% xlim([min(path1.path(:,1))-10 max(path1.path(:,1))+10]);
% ylim([min(path1.path(:,2))-10 max(path1.path(:,2))+10]);
% zlim([min(path1.path(:,3))-10 max(path1.path(:,3))+10]);
% xlabel('x'); ylabel('y'); zlabel('z');
% plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');

for i = 1:size(sensor_list, 2)
    sensor = sensor_list(i);
    color = color_list(i);
    sensor.calculate_measurements(path1.path);
end

%%
% test iterative estimator
import estimation.iterative_estimator;

it = iterative_estimator(sensor_list, path1.path(1,:));

estimated_path = it.estimate_path_by_distance();

%%
% display path vs estimation

% numberOfFrames = length(path1.path) * 2;
numberOfFrames = length(path1.path);
hFigure = figure;
hFigure.Position = [100 100 1000 800];
% Set up the movie structure.
% Preallocate movie, which will be an array of structures.
% First get a cell array with all the frames.
allTheFrames = cell(numberOfFrames,1);
vidHeight = 344;
vidWidth = 446;
allTheFrames(:) = {zeros(vidHeight, vidWidth, 3, 'uint8')};
% Next get a cell array with all the colormaps.
allTheColorMaps = cell(numberOfFrames,1);
allTheColorMaps(:) = {zeros(256, 3)};
% Now combine these to make the array of structures.
myMovie = struct('cdata', allTheFrames, 'colormap', allTheColorMaps);
% Create a VideoWriter object to write the video out to a new, different file.
% writerObj = VideoWriter('problem_3.avi');
% open(writerObj);
% Need to change from the default renderer to zbuffer to get it to work right.
% openGL doesn't work and Painters is way too slow.
set(gcf, 'renderer', 'zbuffer');


sleep_duration = 0.1;

views = [-37.5 30; 90 0; 0 90; 0 0];
for j = 1:4
    subplot(2,2,j)
    hold on 
    grid minor
    view(views(j,:));
    xlim([min(estimated_path(:,1))-80 max(estimated_path(:,1))+80]);
    ylim([min(estimated_path(:,2))-80 max(estimated_path(:,2))+80]);
    zlim([min(estimated_path(:,3))-80 max(estimated_path(:,3))+80]);
    xlabel('x'); ylabel('y'); zlabel('z');
end

for i = 1:size(sensor_list, 2)
    sensor = sensor_list(i);
    color = color_list(i);
    sensor.calculate_measurements(path1.path);
    scatter3(sensor.sensor_position(:,1), sensor.sensor_position(:,2), sensor.sensor_position(:,3), 'filled', color);
end

% plot3(path1.path(:,1), path1.path(:,2), path1.path(:,3), 'b.-');
% plot3(estimated_path(:,1), estimated_path(:,2), estimated_path(:,3), 'r*');

% axis([0 3000 0 3000 0 3000])
for i = 1:size(views,1)
    ax1(j) = plot3(path1.path(1:1,1), path1.path(1:1,2), path1.path(1:1,3), 'b.-');
end
for i = 1:length(path1.path)
    % frameIndex = i * 2 - 1;
    frameIndex = i;
    for j = 1:size(views,1)
        subplot(2,2,j);
        delete(ax1(j));
        ax1(j) = plot3(path1.path(1:i,1), path1.path(1:i,2), path1.path(1:i,3), 'b.-');
    end
    
    prediction_position = estimated_path(i,:);
    for j = 1:size(views,1)
        subplot(2,2,j);
        ax3(j) = plot3(prediction_position(1), prediction_position(2), prediction_position(3), 'r*'); 
    end
    % pause(sleep_duration);
    drawnow;
    thisFrame = getframe(hFigure);
    myMovie(frameIndex) = thisFrame;
    
    for j = 1:size(views,1)
        subplot(2,2,j);
        delete(ax3(j));
        ax3(j) = plot3(prediction_position(1), prediction_position(2), prediction_position(3), 'r.'); 
    end
    % drawnow;
    % thisFrame = getframe(hFigure);
    % frameIndex = i * 2;
    % myMovie(frameIndex) = thisFrame;
end


message = sprintf('Done creating movie\nDo you want to play it?');
button = questdlg(message, 'Continue?', 'Yes', 'No', 'Yes');
drawnow;	% Refresh screen to get rid of dialog box remnants.
close(hFigure);
if strcmpi(button, 'Yes')
	hFigure = figure;
	% Enlarge figure to full screen.
	% set(gcf, 'Units', 'Normalized', 'Outerposition', [0, 0, 1, 1]);
	title('Playing the movie we created', 'FontSize', 15);
	% Get rid of extra set of axes that it makes for some reason.
	axis off;
	% Play the movie.
	movie(myMovie);
	close(hFigure);
end

% See if they want to save the movie to an avi file on disk.
promptMessage = sprintf('Do you want to save this movie to disk?');
titleBarCaption = 'Continue?';
button = questdlg(promptMessage, titleBarCaption, 'Yes', 'No', 'Yes');
if strcmpi(button, 'yes')
	% Get the name of the file that the user wants to save.
	% Note, if you're saving an image you can use imsave() instead of uiputfile().
	startingFolder = pwd;
	defaultFileName = {'*.avi';'*.mp4';'*.mj2'}; %fullfile(startingFolder, '*.avi');
	[baseFileName, folder] = uiputfile(defaultFileName, 'Specify a file');
	if baseFileName == 0
		% User clicked the Cancel button.
		return;
	end
	fullFileName = fullfile(folder, baseFileName);
	% Create a video writer object with that file name.
	% The VideoWriter object must have a profile input argument, otherwise you get jpg.
	% Determine the format the user specified:
	[folder, baseFileName, ext] = fileparts(fullFileName);
	switch lower(ext)
		case '.jp2'
			profile = 'Archival';
		case '.mp4'
			profile = 'MPEG-4';
		otherwise
			% Either avi or some other invalid extension.
			profile = 'Uncompressed AVI';
	end
	writerObj = VideoWriter(fullFileName, profile);
    writerObj.FrameRate = 30;
	open(writerObj);
	% Write out all the frames.
    numberOfFrames = length(myMovie);
	for frameNumber = 1 : numberOfFrames 
        writeVideo(writerObj, myMovie(frameNumber));
	end
	close(writerObj);
end


%%
% test estimator with working and non-working examples

% temp_path = [
%     1.0,1.0,0;
%     0.9,0.9,0;
%     0.8,0.8,0;
%     0.7,0.7,0;
%     0.6,0.6,0;
%     0.5,0.5,0;
%     0.4,0.4,0;
%     0.3,0.3,0;
%     0.2,0.2,0;
%     0.1,0.1,0;
%     0,0,0;
%     ];
% % working
% sensor_pos = [0,0,1; 1,0,0; 0,1,0; 0,0.8,0];
% get_path_from_sensor_pos(sensor_pos, temp_path);
% % figure()
% % plot3(sensor_pos(:,1),sensor_pos(:,2),sensor_pos(:,3));
% % hold on;
% % plot3(0.5,0.5,0, "r.");
% % non working
% temp_path = [
%     0.5,0.5,0.5;
%     0.5,0.5,0;
%     0.5,0.5,-0.5;
%     ];
% %sensor_pos = [0,0,0; 1,0,0; 0,1,0];
% %get_path_from_sensor_pos(sensor_pos, temp_path);

%% functions

% function estimated_path = get_path_from_sensor_pos(sensor_pos, path)
%     import simulation.noisy_sensor;
%     import estimation.iterative_estimator;
%     sensor_list = noisy_sensor.empty();
%     for i = 1:size(sensor_pos, 1)
%         sensor = noisy_sensor(sensor_pos(i,:), "has_distance", true, "has_angle", false, "distance_noise_sigma", 0.1);
%         sensor_list(i) = sensor;
%         sensor.calculate_measurements(path);
%     end

%     it = iterative_estimator(sensor_list, path(1,:));
%     estimated_path = it.estimate_path_by_distance();
%     display(estimated_path);
% end