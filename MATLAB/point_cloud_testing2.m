clear; clc
%This script will retrieve all frames from an .rrf file as fast as possible
%and show a few different visualizations of the point cloud


% retrieve royale version information
royaleVersion = royale.getVersion();
fprintf('* royale version: %s\n',royaleVersion);

%Change this to the file name of the .rrf file in your workspace that you
%wish to read
FileName = 'rrf_output19_35FPS.rrf';


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
%% Get the data out of the rrf file, Warning: These code blocks must be run sequentially
disp('Gathering data...')
for ii = 1:N_Frames
    % retrieve data from camera
    data{ii} = cameraDevice.getData();
end 

%% Render Point Cloud In this Block
%instantiate
clc
cloud_array = cell(1,N_Frames); 
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
disp('Ready to play')
%% play visualization
player = pcplayer([minX maxX],[minY maxY],[minZ maxZ],'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
clc
% camva(player.Axes, 1)
player.Axes.Color = 'k';
% camproj(player.Axes,'perspective')
camva(player.Axes, 5)
% VA = camva(player.Axes);
% player.Axes.Parent
view(player.Axes, [0 -50])


%%%%%%% Legacy 
% set(gca,'visible','off')
% for ii = 1:N_Frames
%     view(player,cloud_array{ii});
%     set(player.Axes.Parent, 'color', 'k')
% %     set(player.Axes, 'axis','off')
%     
%     pause(.05)
%     if ii >= N_Frames
%         ii = 1;
%     end 
% end 


start = 10;
ii=start;
filename = 'brenden_conor_Pico_gif.gif';
while ~0
    view(player,cloud_array{ii});
    set(player.Axes.Parent, 'color', 'k')
%     set(player.Axes, 'axis','off')
    
     pause(.001)
%     if ii >= N_Frames
%         ii = 1;
%     end 
    ii = ii +1; 
        %save to gif, remember CTRL + T to uncomment selection
%     filename = "flexx_heatmap_animation.gif";
%     
    frame = getframe(player.Axes); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if ii == start + 1 
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    elseif mod(ii,5) == 0 
      imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end 
end

%% Merge
clc
% THIS CODE IS FROM THE MATLAB DOCS, COMPUTER VISION: MERGE BY ICP REGISTER
% This only stitches rotational translations, not linear 
%initiate transform
%this will perform one iteration here
disp('Stitching point cloud...')
tic
cloud_array2 = cell(1,N_Frames); %making a new one so this can be used separatly of the movie 
for ii = 1:N_Frames
    cloud_array2{ii} = pointCloud(cloud_array{ii}.Location); 
end 
ptCloudRef = cloud_array{1};
ptCloudCurrent = cloud_array{2};
gridSize = 0.001; %IMPORTANT: this value is how many subgrids are compared when translating
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
ptCloudAligned = pctransform(ptCloudCurrent,tform);
% Note that the downsampling step does not only speed up the registration,
% but can also improve the accuracy.

mergeSize = 0.012; %determines how much of every frame is kept after merge, smaller = more iterations = more points
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
accumTform = tform; 
for i = 3:N_Frames
    ptCloudCurrent = cloud_array{i};
       
    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    
    % Apply ICP registration.
    tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);

    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
    
    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
end
% ptCloudScene = pcdenoise(ptCloudScene);
toc
disp('Merge complete')
%% Show Stitch


%Here is how to set the point cloud color
% It makes sense that it is uniform since, after stitching, heatmaps don't
% make as much sense
%uncomment the next 5 lines to let MATLAB color the cloud
% pointscolor=uint8(zeros(ptCloudScene.Count,3));
% pointscolor(:,1)=0;
% pointscolor(:,2)=255;
% pointscolor(:,3)=51;  
% ptCloudScene.Color = pointscolor; 

pcshow(ptCloudScene,'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
set(gcf, 'color', 'k')
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');
view([0 -90])
axis vis3d

%% Test block

clc
pcshow(cloud_array{20},'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
set(gcf, 'color', 'k')
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');
view([0 -70])
axis vis3d
%% Functions  


%Rotation Function
% Function takes in a single coordinate, and a rotation vector for a
% rotation in degrees about each axis 
function newCoord = rotate(point, theta)  
    theta= theta.*(pi/180); % convert to radians, alternatively, could use sind(degrees)
    %apply about x
    newCoord = [1 0 0; 0 cos(theta(1)) -sin(theta(1)); 0 sin(theta(1)) cos(theta(1))]*point';
    %apply about y
    newCoord = [cos(theta(2)) 0 sin(theta(2)); 0 1 0; -sin(theta(2)) 0 cos(theta(2))]*newCoord;
    %apply about z
    newCoord = ([cos(theta(3)) -sin(theta(3)) 0; sin(theta(3)) cos(theta(3)) 0; 0 0 1]*newCoord)';
end
%% 
% function takes in a ptcloud matrix (Mx3) where M is the number of points
% and applies the specifies rotation to each point in the matrix 
function rotatedMatrix = rotateMatrix(pcld_Location_matrix, theta)
    [nrows, ncols] = size(pcld_Location_matrix); 
    %instantiate
    rotatedMatrix = zeros(nrows,ncols);  
    for point = 1:nrows
        rotatedMatrix(point,:) = rotate(pcld_Location_matrix(point,:), theta); 
    end 

end 