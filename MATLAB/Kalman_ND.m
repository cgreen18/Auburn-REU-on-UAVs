
%state variables
clear; clc; close all; 
%estimate scalar random constant voltage reading from a source
h = figure();

kmax = 10; 

%%%time:pitch:roll:yaw:demo_alt:Vx:Vy:Vz:Mx:My:Mz:alt_vision:alt_raw:Ax:Ay:Az:Wx:Wy:Wz
% var_names = ['time      '; 'pitch     '; 'roll      '; 'yaw       '; 'demo_alt  '; 'Vx        '; 'Vy        '; 'Vz        ';'Mx        ';'My        ';'Mz        ';'alt_vision';'alt_raw   ';'Ax        ';'Ay        ';'Az        ','Wx        ';'Wy        ';'Wz        ']
nav_FileName = 'calibration_ar174_flying_data.txt'; 
nav_data_raw = readtable(nav_FileName, 'Delimiter',':','ReadVariableNames',false);
nav_times = nav_data_raw{:,1};
%indirectly measured values
%z = [0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45 0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45 0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45 0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45]; 
% % % rng('shuffle','twister')
% % % z = (.5-.39).*rand(1000,1) + 0.39; 
z = nav_data_raw{:,1:19};
%% Kalman Filter
%intital error covariance

disp('Processing Kalman Filter')
n = 19; % state size
m = size(z,2); % observation size
% % % % F = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
F = eye(n)
% % % % H = [1 0 0 0; 0 1 0 0];
H = eye(m);
Q = 0.1*eye(n);
% R = 1*eye(m);
R = 0.3*eye(m); % measurement noise covariance
P0 = 10*eye(n); % variance 
%%%%%%%%%%%%%%%%Sensors we want%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % initx = [x, y, z ,v_x, v_y, v_z, roll, pitch  yaw, omega_x, omega_y, omega_z]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initx = [0 0 0 0 0]'; %initial state
initx = zeros(n,1);
% % % initV = 10*eye(n);
% [xfilt, Vfilt] = KalmanFilter(z, F, H, Q, R, initx, initV);
xtrue = KalmanFilter(z, F, H, Q, R, initx, P0);
%grabbing the data
estimate_data = zeros(length(xtrue),n); 
for ii = 1:length(xtrue)
    estimate_data(ii,:) = xtrue{ii}(:)'; 
end 
disp('Kalman Filter Finished')
%% Plot normal
data_choice = 3;
plot(1:length(xtrue), z(:,data_choice),'.','Color', 'r','MarkerSize',10),hold on
plot(1:length(xtrue), estimate_data(:,data_choice),'.','Color', 'b','MarkerSize',10) 
% % % comet(1:length(xtrue), z(:,data_choice)),hold on
% % % comet(1:length(xtrue), estimate_data(:,data_choice)), hold off
legend({'Measured','Kalman Estimate'},'Location','Southeast')
% set(gcf, 'color', 'k')
% set(gca,'color','k')
% set(gca,'ycolor','r')
% set(gca,'xcolor','r')
%% Still subplots 
clc
h = figure('units','normalized','outerposition',[0 0 1 1]);
ax = gca;
set(gcf, 'color', 'k')

data_choice = [3 4 6 7]; %the indicies out of nav_data_raw
num_data = length(data_choice);
marker_size = 6; 

kalman_color = 'b';
measured_color = 'r';
for ii = 1:num_data
    subplot(2,2,ii)
    plot(1:length(xtrue), z(:,data_choice(ii)),'.','Color', measured_color,'MarkerSize',marker_size), hold on
    plot(1:length(xtrue), estimate_data(:,data_choice(ii)),'.','Color', kalman_color,'MarkerSize',marker_size) 
    set(gca,'color','k'); xlabel('\fontsize{16}t {samples}'); ylabel('\fontsize{16}degrees')
    set(gca,'ycolor','r'); 
    set(gca,'xcolor','r')
%     set(subplot(2,2,ii), 'ylim', [min([min(z(:,data_choice(ii))), min(estimate_data(:,data_choice(ii)))]) max([max(z(:,data_choice(ii))), max(estimate_data(:,data_choice(ii)))])]);
%     set(subplot(2,2,ii), 'xlim', [0 length(xtrue)]);
end 




%% Movie Visualize Plot (Still working on this) 
clc
h = figure('units','normalized','outerposition',[0 0 1 1]);
ax = gca;
set(gcf, 'color', 'k')

% Subplot properties
marker_size = 8; 
kalman_color = 'r';
measured_color = 'b';
begin_at = 1; 
end_at = 1000;
skip_points =1; 
ii = begin_at;
replay = true; 
background_color = 'k'; %k = black
axis_color = 'r';

%init subplots
subplot(2,2,1)
d_choice1 = data_choice(1);  %change this to change what variable is shown on the subplot
title('\fontsize{18}\color{red}Yaw Evolution')
set(gca,'color',background_color); xlabel('\fontsize{16}t {samples}'); ylabel('\fontsize{16}degrees')
set(gca,'ycolor',axis_color); 
set(gca,'xcolor',axis_color)
set(subplot(2,2,1), 'ylim', [min([min(z(:,d_choice1)), min(estimate_data(:,d_choice1))]), max([max(z(:,d_choice1)), max(estimate_data(:,d_choice1))])]);
set(subplot(2,2,1), 'xlim', [0 end_at]);
hold on
% axis([0 length(xtrue) min(z(:,2)) max(z(:,2))])

subplot(2,2,2)
d_choice2 = data_choice(2); 
title('\fontsize{18}\color{red}Roll Evolution')
set(gca,'color',background_color); xlabel('\fontsize{16}t {samples}'); ylabel('\fontsize{16}degrees')
set(gca,'ycolor',axis_color); 
set(gca,'xcolor',axis_color)
set(subplot(2,2,2), 'ylim', [min([min(z(:,d_choice2)), min(estimate_data(:,d_choice2))]), max([max(z(:,d_choice2)), max(estimate_data(:,d_choice2))])]);
set(subplot(2,2,2), 'xlim', [0 end_at]);
hold on

subplot(2,2,3)
d_choice3 = data_choice(3); 
title('\fontsize{18}\color{red}X Evolution')
set(gca,'color',background_color); xlabel('\fontsize{16}t {samples}'); ylabel('\fontsize{16}distance (mm)')
set(gca,'ycolor',axis_color); 
set(gca,'xcolor',axis_color)
set(subplot(2,2,3), 'ylim', [min([min(z(:,d_choice3)), min(estimate_data(:,d_choice3))]), max([max(z(:,d_choice3)), max(estimate_data(:,d_choice3))])]);
set(subplot(2,2,3), 'xlim', [0 end_at]);
hold on

subplot(2,2,4)
d_choice4 = data_choice(4); 
title('\fontsize{18}\color{red}Y Evolution')
set(gca,'color',background_color); xlabel('\fontsize{16}t {samples}'); ylabel('\fontsize{16}distance (mm)')
set(gca,'ycolor',axis_color); 
set(gca,'xcolor',axis_color)
set(subplot(2,2,4), 'ylim', [min([min(z(:,d_choice4)), min(estimate_data(:,d_choice4))]), max([max(z(:,d_choice4)), max(estimate_data(:,d_choice4))])]);
set(subplot(2,2,4), 'xlim', [0 end_at]);
hold on


% for markers use markersize, for lines use linewidth
  %start location
while 1
    if ~ishandle(h)
        break
    end 
    hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Subplot 1 
    subplot(2,2,1)
    plot(ii, z(ii,d_choice1),'.','Color', measured_color,'MarkerSize',marker_size)
    plot(ii, estimate_data(ii,d_choice1),'.','Color', kalman_color,'MarkerSize',marker_size) 
%     drawnow
%     hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Subplot 2
    subplot(2,2,2)
    plot(ii, z(ii,d_choice2),'.','Color', measured_color,'MarkerSize',marker_size)
    plot(ii, estimate_data(ii,d_choice2),'.','Color', kalman_color,'MarkerSize',marker_size) 
%     drawnow
%     hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Subplot 3
    subplot(2,2,3)
    plot(ii, z(ii,d_choice3),'.','Color', measured_color,'MarkerSize',marker_size)
    plot(ii, estimate_data(ii,d_choice3),'.','Color',kalman_color,'MarkerSize',marker_size) 
%     drawnow
%     hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Subplot 4
    subplot(2,2,4)
    plot(ii, z(ii,d_choice4),'.','Color', measured_color,'MarkerSize',marker_size)
    plot(ii, estimate_data(ii,d_choice4),'.','Color', kalman_color,'MarkerSize',marker_size) 
    hold off
    drawnow
    if ~ishandle(h)
        break
    end 
    if ~(ii >= end_at)
        ii = ii + 1 + skip_points;
    elseif replay
        ii = begin_at;
        for jj= 1:4
            cla(subplot(2,2,jj))
        end 
    else 
        break
    end 
%     pause(.01)
end 
%%
function state = KalmanFilter(z, A, H, Q, R, X0, P0)
    %state init
     X_star = cell(size(X0,1),1);
     P_star = cell(size(P0,1),1);
     X = cell(size(X0,1),1);
     P = cell(size(P0,1),1); 
for k = 1:size(z,1)
    %Future Projection
    if k == 1
        X_star{k} = A*X0; 
        P_star{k} = A*P0*A' + Q;
    else
        X_star{k} = A*X_star{k-1};
        P_star{k} = A*P{k-1}*A' + Q;
        
    end 
    
    %measurement update
    %update gain
    K = P_star{k}*H'*inv(H*P_star{k}*H' + R); 
    X{k} = X_star{k} + K*(z(k,:)' - H*X_star{k}); 
    P{k} = (eye(size(X0)) - K*H)*P_star{k};
    
end 
state = X; 
end 