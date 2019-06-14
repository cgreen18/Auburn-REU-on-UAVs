clear; clc
%This script will retrieve all frames from an .rrf file as fast as possible
%and show a few different visualizations of the point cloud


% retrieve royale version information
royaleVersion = royale.getVersion();
fprintf('* royale version: %s\n',royaleVersion);

%Change this to the file name of the .rrf file in your workspace that you
%wish to read
FileName = 'rrf_output4.rrf';


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
%% Use this to view an individual frame,
% modify frame choice based on how many frames you have in the variable
% N_Frames

figure('units','normalized','outerposition',[0 0 1 1])
set(gcf, 'color', 'k')
frameChoice = 3; 
[X,Y] = meshgrid(1:171,1:224); 
plot3(X',Y',data{frameChoice}.grayValue, '.', 'color', 'g')
axis vis3d; rotate3d on;
%beautify
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');
view(90, 70)

%% Point cloud function??
clc
PCLD = single(zeros(171,224,3)); 
PCLD(:,:,1) = meshgrid(1:171,1:224)';
PCLD(:,:,2) = meshgrid(1:171,1:224)';
PCLD(:,:,3) = data{frameChoice}.depthConfidence;
cloud = pointCloud(PCLD); 

