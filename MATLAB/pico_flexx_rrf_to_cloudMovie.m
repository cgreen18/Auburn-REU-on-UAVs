clear; clc
% % This script will retrieve all frames from a 
% % .rrf produced by the PICO_FLEXX or MATLAB scripts 
% % that are intended to output a .rrf file

% retrieve royale version information
royaleVersion = royale.getVersion();
fprintf('* royale version: %s\n',royaleVersion);
%Change this to the file name of the .rrf file in your workspace that you
%wish to read
FileName = 'rrf_output.rrf';
% open recorded file
manager = royale.CameraManager();
cameraDevice = manager.createCamera(FileName);
delete(manager);
cameraDevice.initialize();

% configure playback
% cameraDevice.loop(false);
 cameraDevice.useTimestamps(false);

N_Frames=cameraDevice.frameCount();

% start capture mode
cameraDevice.startCapture();

%initialize array that will hold all of the frames 
data = cell(1,N_Frames); 
disp('File initiated')

% Get the data out of the rrf file
disp('Retrieving data from file...')
for ii = 1:N_Frames
    % retrieve data from camera
    data{ii} = cameraDevice.getData();
end 
disp('Data stored.')
% Render Point Cloud In this Block
%instantiate
cloud_array = cell(1,N_Frames); 
% color_array = cell(1,N_Frames); 
tic
disp('Rendering clouds, denoising, and processing for player...')
for ii = 1:N_Frames
    XI = reshape(data{ii}.x, [1,38304]);
    YI = reshape(data{ii}.y, [1,38304]);
    ZI = reshape(data{ii}.z, [1,38304]);
    cloud_array{ii} = pointCloud([XI; YI; ZI]');
    
    cloud_array{ii}.Intensity = single(reshape(data{ii}.grayValue,[1,38304]))';
    cloud_array{ii} = pcdenoise(cloud_array{ii}, 'NumNeighbors', 5,'Threshold', 1); % de-noise, uncommenting this line will produce faster but messier clouds
%     color_array{ii} =  reshape(data{ii}.grayValue,[1,38304]);
end 
%%%% pcdenoise(cloud,'NumNeighbors',4, 'Threshold',.1); %keeping for
%%%% reference to denoise

% Get max viewing for pc player axis
maxX_arr = zeros(1,N_Frames);
minX_arr = zeros(1,N_Frames);
maxY_arr = zeros(1,N_Frames);
minY_arr = zeros(1,N_Frames);
maxZ_arr = zeros(1,N_Frames);
minZ_arr = zeros(1,N_Frames);
for ii = 2:N_Frames
    maxX_arr(1,ii) = max(data{ii}.x(:)); 
    minX_arr(1,ii) = min(data{ii}.x(:)); 
    maxY_arr(1,ii) = max(data{ii}.y(:)); 
    minY_arr(1,ii) = min(data{ii}.y(:)); 
    maxZ_arr(1,ii) = max(data{ii}.z(:)); 
    minZ_arr(1,ii) = min(data{ii}.z(:)); 
    %set maximums
    maxX = max(maxX_arr); 
    minX = min(minX_arr); 
    maxY = max(maxY_arr); 
    minY = min(minY_arr);
    maxZ = max(maxZ_arr); 
    minZ = min(minZ_arr); 
end 
toc
disp('Playing...')

%PLAY
%Run this block separately to play
player = pcplayer([minX maxX],[minY maxY],[minZ maxZ],'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
clc
% camva(player.Axes, 1)
player.Axes.Color = 'k';
% camproj(player.Axes,'perspective')
camva(player.Axes, 4.5)
% VA = camva(player.Axes);
% player.Axes.Parent

%%%%%% Legacy 

% for ii = 1:N_Frames
%     view(player,cloud_array{ii});
%     set(player.Axes.Parent, 'color', 'k')
% %     set(player.Axes, 'axis','off')
%     
%     pause(.05)
% end 

view(player.Axes, [0 -65])
ii = 1; 
while ~0
    view(player,cloud_array{ii});
    set(player.Axes.Parent, 'color', 'k')

     pause(.05)
     %replay
    if ii >= N_Frames
        ii = 1;
    end 
    ii = ii +1; 
end

% CTRL + C IN COMMAND WINDOW TO KILL 