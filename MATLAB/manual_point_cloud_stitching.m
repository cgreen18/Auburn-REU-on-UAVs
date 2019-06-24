clear; clc
%This script will retrieve all frames from an .rrf file as fast as possible
%and show a few different visualizations of the point cloud


% retrieve royale version information
royaleVersion = royale.getVersion();
fprintf('* royale version: %s\n',royaleVersion);

%Change this to the file name of the .rrf file in your workspace that you
%wish to read
pico_FileName = 'flexx_sync_4.rrf';
nav_FileName = 'pos_and_eul_data.txt'; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Init Camera
% open recorded file
manager = royale.CameraManager();
cameraDevice = manager.createCamera(pico_FileName);
delete(manager);
cameraDevice.initialize();
% configure playback
% cameraDevice.loop(false);
 cameraDevice.useTimestamps(true);
N_Frames=cameraDevice.frameCount();
% start capture mode
cameraDevice.startCapture();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%End Init Camera Data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Init Navigation Data
% nav_data = textscan(nav_FileName, '%f%f%f%f', 'Delimiter', ':','TreatAsEmpty','~');
nav_data_raw = readtable(nav_FileName, 'Delimiter',':','ReadVariableNames',false);
nav_times = nav_data_raw{:,1};
% fclose(nav_FileName);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%End Navigation Data

% init variables to hold data  
data = cell(1,N_Frames); 
pico_times = zeros(N_Frames,2); 



disp('File initiated')
%% Get the data out of each file, Warning: These code blocks must be run sequentially
disp('Gathering data...')
for ii = 1:N_Frames
    % retrieve data from camera
    data{ii} = cameraDevice.getData();
    % retrieve times
    pico_times(ii) = data{ii}.timeStamp/(10^6); 
end 

% fix pico times
time_multiplier = 1; 
fixed_times = zeros(N_Frames,1);
for ii = 2:N_Frames
    if pico_times(ii,1) == pico_times(ii-1,1)
        fixed_times(ii) = pico_times(ii,1) + 0.2*time_multiplier; 
        time_multiplier = time_multiplier + 1; 
    else
        fixed_times(ii) = pico_times(ii,1);
        time_multiplier = 1; 
    end
end
% delete(fixed_times)
pico_times(:,1) = fixed_times; 
disp('Data stored.')
%% Time sync position with camera data 
clc
% pico_times(:,2) = [];
pico_times(6,1) - nav_times(41)
threshold = .05;
previous_threshold = threshold; %seconds 
differenceFrames = zeros(N_Frames, 6); 
%init total navigation date columns are syncronized time, yaw, pitch, roll
synced_navigation_data = zeros(N_Frames, 7); 

for ii = 1:N_Frames 
    current_time = pico_times(ii,1); 
    for jj = 1:length(nav_times)
        if abs(current_time - nav_times(jj)) <= previous_threshold
            %save the index of the syncronized nav_data
%             disp(jj)
            pico_times(ii,2) = jj; %% the second peice here was for debugging
            synced_navigation_data(ii,1) = current_time;
            synced_navigation_data(ii,2:7) = nav_data_raw{jj,2:7};
%             synced_navigation_data(ii,2:4) = synced_navigation_data(ii,2:4) - synced_navigation_data(ii-1,2:4);
            differenceFrames(ii,:) = synced_navigation_data(ii,2:7) - synced_navigation_data(ii-1,2:7);
            previous_threshold = abs(current_time - nav_times(jj)); 
        end
        
        % here grab the actual nav data that correlates in time with the
        % camera frame
        
    end 
    previous_threshold = threshold;
end 

%% Get the rotation matricies from the euler angles for each frame
clc
% saving all of these is not necessary, used for debugging purposes

rotation_from_euler = cell(1,N_Frames);
translations_from_positions = cell(1,N_Frames);
total_transformation_matrix = cell(1,N_Frames); 
%position millimeter to meter converstion
conversion_factor = 10^3; 

for ii = 1:N_Frames
    eul = differenceFrames(ii,1:3);
    rotation_from_euler{ii} = horzcat(vertcat(euler2rot(eul),[0 0 0]),[0 0 0 1]'); 
    isRigid(affine3d(rotation_from_euler{ii}))
    translations_from_positions{ii} = inv(horzcat(vertcat(eye(3),[0 0 0]),[differenceFrames(ii,4)/conversion_factor differenceFrames(ii,5)/conversion_factor differenceFrames(ii,6)/conversion_factor 1]'));
    total_transformation_matrix{ii} = affine3d((rotation_from_euler{ii}*translations_from_positions{ii})');
    
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
    temp_cloud = [XI; YI; ZI]';
    %apply rotation matrix here
%     for jj = 1:38304
%        temp_cloud(jj,:) = (rotation_from_euler{ii}*(temp_cloud(jj,:)'))'; %ROTATE
%     end
    cloud_array{ii} = pointCloud(temp_cloud);
    
    cloud_array{ii}.Intensity = single(reshape(data{ii}.grayValue,[1,38304]))';
    cloud_array{ii} = pcdenoise(cloud_array{ii}, 'NumNeighbors', 1,'Threshold', .01); % de-noise, uncommenting this line will produce faster but messier clouds
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
%% Manual Merge Testing
clc; 
% Init transform

custom_tform = [1 0 0 0;
                0 1 0 0
                0 0 1 0
                0 0 0 1];
init_transform = total_transformation_matrix{71};

ptCloudRef = cloud_array{70};
ptCloudCurrent = cloud_array{71};
gridSize = 1; %IMPORTANT: this value is how many subgrids are compared when translating
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent,'gridAverage', gridSize);
tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true,'Tolerance',[0.001 0.001]);
ptCloudAligned = pctransform(ptCloudCurrent,tform);
% % % % ptCloudAligned = pctransform(ptCloudCurrent,affine3d(eye(4)));
% Note that the downsampling step does not only speed up the registration,
% but can also improve the accuracy.

mergeSize = 0.01; %determines how much of every frame is kept after merge, smaller = more iterations = more points
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
% accumTform = tform; 


%Here is how to set the point cloud color
% It makes sense that it is uniform since, after stitching, heatmaps don't
% make as much sense
%uncomment the next 5 lines to let MATLAB color the cloud
pointscolor=uint8(zeros(ptCloudScene.Count,3));
pointscolor(:,1)=0;
pointscolor(:,2)=255;
pointscolor(:,3)=51;  
ptCloudScene.Color = pointscolor; 

% pcshow(pcmerge(cloud_array{1}, cloud_array{2}, mergeSize),'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
pcshow(ptCloudScene,'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
set(gcf, 'color', 'k')
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');
view([0 -90])
axis vis3d
%% Manual Merge Testing 2
clc; 
% Init transform

custom_tform = [1 0 0 0;
                0 1 0 0
                0 0 1 0
                0 0 0 1];

% h = figure; 
p = 3; 
q = 50;
ptCloudRef = cloud_array{p};
% ptCloudCurrent = cloud_array{71};
% init_transform = total_transformation_matrix{71};

gridSize = .01; %IMPORTANT: this value is how many subgrids are compared when translating
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
% tform = pcregistericp(moving, fixed, 'Metric','pointToPoint','Extrapolate', true,'Tolerance',[0.001 0.001],'InitialTransform',init_transform);
% % % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% icp function
% % % % q = ptCloudRef.Location'; 
% % % % p = ptCloudCurrent.Location';
% % % % [R,T] = icp(ptCloudRef.Location', ptCloudCurrent.Location', 10,'Matching', 'kDtree', 'Extrapolation', true)
% % % % transformed_cloud = pointCloud((R*p + repmat(T,1,length(p)))');

ptCloudAligned = pctransform(ptCloudRef,total_transformation_matrix{p+1});
for ii = p+2:q
    ptCloudAligned = pctransform(ptCloudAligned,total_transformation_matrix{ii});
end
mergeSize = 0.01; %determines how much of every frame is kept after merge, smaller = more iterations = more points
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
% accumTform = tform; 

%Here is how to set the point cloud color
% It makes sense that it is uniform since, after stitching, heatmaps don't
% make as much sense
%uncomment the next 5 lines to let MATLAB color the cloud
pointscolor=uint8(zeros(ptCloudScene.Count,3));
pointscolor(:,1)=0;
pointscolor(:,2)=255;
pointscolor(:,3)=51;  
ptCloudScene.Color = pointscolor; 

% pcshow(pcmerge(cloud_array{1}, cloud_array{2}, mergeSize),'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
pcshow(ptCloudScene,'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
set(gcf, 'color', 'k')
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');
view([0 -90])
axis vis3d

%% ICP Register Merge Full (Not working properly yet)
% NOTE: Works when rotated perfectly around camera y-axis, see
% rrf_output13_rotate90_5FPS.rrf for best results
clc
% THIS CODE IS FROM THE MATLAB DOCS, COMPUTER VISION: MERGE BY ICP REGISTER
% This only stitches rotational translations, not linear 
%this will perform one iteration here
disp('Stitching point cloud...')
tic
% Init transform

init_transform = affine3d(translations_from_positions{3}');

ptCloudRef = cloud_array{2};
ptCloudCurrent = cloud_array{3};
gridSize = 0.001; %IMPORTANT: this value is how many subgrids are compared when translating
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
% % % % tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true,'InitialTransform',init_transform);
tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);

ptCloudAligned = pctransform(ptCloudCurrent,tform);
% Note that the downsampling step does not only speed up the registration,
% but can also improve the accuracy.

mergeSize = .035; %determines how much of every frame is kept after merge, smaller = more iterations = more points
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
accumTform = tform; 
for ii = 3:N_Frames
    ptCloudCurrent = cloud_array{ii};
       
    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    
    %get next measured frame transform
%     init_transform = affine3d(translations_from_positions{ii}');
    
    % Apply ICP registration.
    tform = pcregistericp(moving, fixed, 'Metric','pointToPoint','Tolerance',[0.0001, 0.0005]);

    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
% % % %     accumTform = affine3d(init_transform.T * accumTform.T);
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
pointscolor=uint8(zeros(ptCloudScene.Count,3));
pointscolor(:,1)=0;
pointscolor(:,2)=255;
pointscolor(:,3)=51;  
ptCloudScene.Color = pointscolor; 

pcshow(ptCloudScene,'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
set(gcf, 'color', 'k')
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');
view([0 -90])
axis vis3d


%% Euler to rotation matrix 
function rot_matrix = euler2rot(angles)
    phi = angles(1) *pi/180; theta = angles(2) *pi/180; psi = angles(3) *pi/180; 
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1]; 
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rx = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
    rot_matrix = Rz*Ry*Rx; 


end 
