clear; clc
%This script will retrieve all frames from an .rrf file as fast as possible
%and show a few different visualizations of the point cloud


% retrieve royale version information
royaleVersion = royale.getVersion();
fprintf('* royale version: %s\n',royaleVersion);

%Change this to the file name of the .rrf file in your workspace that you
%wish to read
FileName = 'rrf_output7_5FPS.rrf';


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


%% Get the data out of the rrf file, Warning: These code blocks must be run sequentially

for ii = 1:N_Frames
    % retrieve data from camera
    data{ii} = cameraDevice.getData();
end 

%% Render Point Cloud In this Block
%instantiate
clc
cloud_array = cell(1,N_Frames); 
% color_array = cell(1,N_Frames); 
for ii = 1:N_Frames
    XI = reshape(data{ii}.x, [1,38304]);
    YI = reshape(data{ii}.y, [1,38304]);
    ZI = reshape(data{ii}.z, [1,38304]);
    cloud_array{ii} = pointCloud([XI; YI; ZI]');
    cloud_array{ii}.Intensity = single(reshape(data{ii}.grayValue,[1,38304]))';
%     color_array{ii} =  reshape(data{ii}.grayValue,[1,38304]);
end 
%% play
player = pcplayer([min(data{100}.x(:)) 2],[min(data{100}.y(:)) 2],[min(data{100}.z(:)) 3]);
clc
% camva(player.Axes, 1)
player.Axes.Color = 'k';
% camproj(player.Axes,'perspective')
camva(player.Axes, 3.8)
% VA = camva(player.Axes);
% player.Axes.Parent
view(player.Axes, [45 40])
% set(gca,'visible','off')
for ii = 1:N_Frames
    view(player,cloud_array{ii});
    set(player.Axes.Parent, 'color', 'k')
%     set(player.Axes, 'axis','off')
    
    pause(.1)
end 

%% Merge?? (Working on this)
clc
for ii = 2:N_Frames
    merge = pcmerge(cloud_array{ii},cloud_array{ii-1},.01); 
    
end
pcshow(merge)
set(gcf, 'color', 'k')
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');
view(90,45)
axis vis3d