clear; clc
%This script will retrieve all frames from an .rrf file as fast as possible
%and show a few different visualizations of the point cloud


% retrieve royale version information
royaleVersion = royale.getVersion();
fprintf('* royale version: %s\n',royaleVersion);

%Change this to the file name of the .rrf file in your workspace that you
%wish to read
FileName = 'rrf_sample_output.rrf';


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
frameChoice = 50; 
[X,Y] = meshgrid(1:171,1:224); 
plot3(X',Y',data{frameChoice}.grayValue, '.', 'color', 'g')
axis vis3d; rotate3d on;
%beautify
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');
view(90, 70)
%% plot 3D movie
%play through the frames in Data

h = figure('units','normalized','outerposition',[0 0 1 1]);
%beautify
set(gcf, 'color', 'k')

for ii = 1:N_Frames
    if ~ishandle(h)
        return 
    end 
%%%%%%%%%%%%%%%%%%%%%%%%%%% Movie 
    maxZ = max(data{ii}.grayValue(:)); %get the max of the entire matrix 
    p = plot3((X'),(Y'),data{ii}.grayValue,'.','color','g','MarkerSize',1);
    axis([0 171 0 224 0 3600])
    view([90, 68])
    set(gca, 'color', 'k'); 
    set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
    set(gca, 'zcolor', 'b'); 
    axis vis3d; rotate3d on
    drawnow;
    delete(p)
    %pause(.1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

%close camera stream 
cameraDevice.stopCapture();
fprintf('* ...done!\n');

%% colored scatter plotting
% Here we want to "linearize" the matricies so that they can be scatter
% plotted by MATLAB. This way, we can then linearly map a heatmap

clc
figure('units','normalized','outerposition',[0 0 1 1])
set(gcf, 'color', 'k')
% frameChoice = 50; 
[X,Y] = meshgrid(1:171,1:224); 

axis vis3d; rotate3d on;

XS = reshape(X', 1, 38304); YS = reshape(Y', 1,38304);
ZS = reshape(data{40}.grayValue, [1,38304]); 
C = reshape(data{40}.grayValue,[1,38304]); 
scatter3(XS,YS,ZS,1,C)
%beautify
colormap(jet); colorbar
axis vis3d; rotate3d
set(gca, 'color', 'k'); 
set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
set(gca, 'zcolor', 'b');

%adjust this to change the view programmatically, (azimuth, elevation)
view(90, 70)

%% Scatter 3d Movie

clc
h = figure('units','normalized','outerposition',[0 0 1 1])
set(gcf, 'color', 'k')
frameChoice = 50; 
[X,Y] = meshgrid(1:171,1:224); 
XS = reshape(X', 1, 38304); YS = reshape(Y', 1,38304);
for ii = 1:80
    if ~ishandle(h)
        return 
    end 
    
    axis vis3d; 

    ZS = reshape(data{ii}.grayValue, [1,38304]); 
    C = reshape(data{ii}.grayValue,[1,38304]); 
    scatter3(XS,YS,ZS,1,C)
    axis([0 171 0 224 0 3600])
    %beautify
    colormap(jet); colorbar
    axis vis3d; rotate3d
    set(gca, 'color', 'k'); 
    set(gca, 'xcolor', 'b'); set(gca, 'ycolor', 'b')
    set(gca, 'zcolor', 'b');
    %adjust this to change the view programmatically, (azimuth, elevation)
    view(90, 55)
    drawnow
    
    %save to gif, remember CTRL + T to uncomment selection
%     filename = "flexx_heatmap_animation.gif";
%     
%     frame = getframe(h); 
%     im = frame2im(frame); 
%     [imind,cm] = rgb2ind(im,256); 
%     % Write to the GIF File 
%     if ii == 1 
%       imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     elseif mod(ii,2) == 0 
%       imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%     end 
    
    
end