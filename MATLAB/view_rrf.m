clear; clc
%This script will retrieve all frames from an .rrf file as fast as possible
%and show a few different visualizations of the point cloud


% retrieve royale version information
royaleVersion = royale.getVersion();
fprintf('* royale version: %s\n',royaleVersion);

FileName = 'rrf_output4.rrf'; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Change me

% open recorded file
manager = royale.CameraManager();
cameraDevice = manager.createCamera(FileName);
delete(manager);
cameraDevice.initialize();
cameraDevice.useTimestamps(true);
N_Frames=cameraDevice.frameCount();
cameraDevice.startCapture();

%initialize array that will hold all of the frames 
data = cell(1,N_Frames); 

% Render Point Cloud In this Block
%instantiate
clc
cloud_array = cell(1,N_Frames); 
disp('Gathering data...')
for ii = 1:N_Frames
    % retrieve data from camera
    data{ii} = cameraDevice.getData();
end 
disp('Data stored.')
% color_array = cell(1,N_Frames); 
tic
disp('Generating point clouds, denoising, and processing player...')
for ii = 1:N_Frames
    XI = reshape(data{ii}.x, [1,38304]);
    YI = reshape(data{ii}.y, [1,38304]);
    ZI = reshape(data{ii}.z, [1,38304]);
    cloud_array{ii} = pointCloud([XI; YI; ZI]');
    
    cloud_array{ii}.Intensity = single(reshape(data{ii}.grayValue,[1,38304]))';
    cloud_array{ii} = pcdenoise(cloud_array{ii}, 'NumNeighbors', 5,'Threshold', 1); % de-noise, uncommenting this line will produce faster but messier clouds
%     color_array{ii} =  reshape(data{ii}.grayValue,[1,38304])
end 
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
disp('Ready to play')

%% play visualization of entire movie
player = pcplayer([minX maxX],[minY maxY],[minZ maxZ],'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
clc
% camva(player.Axes, 1)
player.Axes.Color = 'k';
% camproj(player.Axes,'perspective')
camva(player.Axes, 4.5)
% VA = camva(player.Axes);
% player.Axes.Parent
view(player.Axes, [0 -90])


%%%%%% Legacy 

% for ii = 1:N_Frames
%     view(player,cloud_array{ii});
%     set(player.Axes.Parent, 'color', 'k')
% %     set(player.Axes, 'axis','off')
%     
%     pause(.05)
% end 
view(player.Axes, [0 -65])

% start = 10;
ii = 1; 
% filename = 'brenden_conor_Pico_gif.gif';
while ~0
    view(player,cloud_array{ii});
    set(player.Axes.Parent, 'color', 'k')
%     set(player.Axes, 'axis','off')
%     view(player.Axes, [0 -65 - ii/10])
     pause(.05)
     %replay
    if ii >= N_Frames
        ii = 1;
    end 
    ii = ii +1; 
    pause(.01)
%   
    if ~ishandle(player)
%         break
        
    end 
   
end