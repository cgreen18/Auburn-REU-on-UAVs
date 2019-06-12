%% Parsing the data from the pico flexx, You must run this block to run the rest
% We saved the pico flexx data where each row correlates to a row in the
% This program is set up in code blocks and meant to be run individually


%data matrix
clear, clc
camResX = 171; camResY = 224; 
rawData = readtable('data.txt');
[Width, Height] = size(rawData);
if Height == 225
    rawData = removevars(rawData,{'Var225'});
    Height = Height - 1;
end

numFrames = Width*Height/(camResX*camResY);
%msg = 'There are ' + num2str(numFrames) + ' of depth data'; 
%disp(msg)

%Here we will grab each frame from the raw data 
frames = cell(1,numFrames); 
for ii = 1:numFrames-1
    frames{1,ii} = rawData{(ii)*171+1:(ii)*camResX + camResX,1:end}; 
end 

%% surface plot NOTE: You must run the first code black to run all others
clc
frameChoice = 38; 
%create foundation to plot above
[X, Y] = meshgrid(1:camResY,1:camResX); 
%grab the depth to plot
Z = frames{frameChoice}(:,:);

%maximize and beautify window
figure('units','normalized','outerposition',[0 0 1 1])
rotate3d on; set(gcf,'color', 'k'); set(gca,'color','k')
set(gca,'ycolor','r'); 
set(gca,'xcolor','r'); 

%plot 
surf(X,Y,Z) %4th arg is color 
axis vis3d
axis off
colormap
%view([0 90]) %azimuth and elevation

%% You can play a movie with this code block
%maximize and beautify window
figure('units','normalized','outerposition',[0 0 1 1])
rotate3d on; set(gcf,'color', 'k'); set(gca,'color','k')
set(gca,'ycolor','r'); 
set(gca,'xcolor','r'); 
view([0 90]) %azimuth and elevation

for ii = 1:numFrames-1
    frameChoice = ii; 
    
    %grab the depth to plot
    Z = frames{frameChoice}(:,:);
    
    %plot 
    s = surf(X,Y,Z); %4th arg is color 
    axis vis3d
    axis off
    colormap
    view(gca,[0 90]) %azimuth and elevation, adjust this to adjust the view 
    drawnow
    pause(.05)
end

%% (Point cloud??) (Currently Not Working well) 
frameChoice = 10; 
%maximize and beautify window
figure('units','normalized','outerposition',[0 0 1 1])


tic

%This block if you have a GPU
% for xi = gpuArray(1:camResX)
%     for yi = gpuArray(1:camResY)
%         plot3(gpuArray(xi),gpuArray(yi),gpuArray(frames{frameChoice}(xi,yi)),'.','color','g')
%         hold on
%     end
% end

%This block without a GPU
for xi = gpuArray(1:camResX)
    for yi = gpuArray(1:camResY)
        plot3(gpuArray(xi),gpuArray(yi),gpuArray(frames{frameChoice}(xi,yi)),'.','color','g')
        hold on
    end
end

hold off
axis vis3d; 
rotate3d on; set(gcf,'color', 'k'); set(gca,'color','k')
set(gca,'zcolor','r');
set(gca,'ycolor','r'); 
set(gca,'xcolor','r'); 
toc


