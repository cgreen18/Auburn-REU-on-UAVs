clear; clc; close all; 
%state variables

%kmax = 10; 
%nav_FileName = 'pos_and_eul_data.txt'; 
%nav_data_raw = readtable(nav_FileName, 'Delimiter',':','ReadVariableNames',false);
%nav_times = nav_data_raw{:,1};
%indirectly measured values
%z = [0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45 0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45 0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45 0.39 0.50 0.48 0.29 0.25 0.32 0.34 0.48 0.41 0.45]; 
% % % rng('shuffle','twister')
% % % z = (.5-.39).*rand(1000,1) + 0.39; 
% % % z = nav_data_raw{:,2:6};

global num_data_pts 
num_data_pts = 300

global num_sens
num_sens = 12
global num_states
num_states = 12

global delta_t 
delta_t = 1/200;

z = zeros(num_data_pts,num_sens);

%Test at Vx=5
z(:,4) = 5; 
z(:,1) = linspace(0,5*delta_t*num_data_pts,num_data_pts);
%intital error covariance


n = num_states; % state size
m = num_sens; % observation size
% % % % F = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
% % % % F = eye(n)
R_1 = @(psi)[cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
R_2 = @(theta) [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R_3 = @(phi) [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
R_b__n = @(psi , theta , phi) R_1(psi)*R_2(theta)*R_3(phi);

Rbn = R_b__n(0,0,0);
F= [1,0,0,delta_t,0,0,0,0,0,0,0,0;
    0,1,0,0,delta_t,0,0,0,0,0,0,0;
    0,0,1,0,0,delta_t,0,0,0,0,0,0;
    0,0,0,1,0,0,0,0,0,0,0,0;
    0,0,0,0,1,0,0,0,0,0,0,0;
    0,0,0,0,0,1,0,0,0,0,0,0;
    0,0,0,0,0,0,1,0,0,0,0,0;
    0,0,0,0,0,0,0,1,0,0,0,0;
    0,0,0,0,0,0,0,0,1,0,0,0;
    0,0,0,0,0,0,0,0,0,1,0,0;
    0,0,0,0,0,0,0,0,0,0,1,0;
    0,0,0,0,0,0,0,0,0,0,0,1];

%F(7:9 , 10:12) = Rbn*delta_t;



% % % % H = [1 0 0 0; 0 1 0 0];
H = eye(m);
Q = 0.6*eye(n);
% R = 1*eye(m);
R = 0.6*eye(m); % measurement noise covariance
P0 = .3*eye(n); % variance 
%%%%%%%%%%%%%%%%Sensors we want%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % initx = [x, y, z ,v_x, v_y, v_z, roll, pitch  yaw, omega_x, omega_y, omega_z]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
initx = zeros(num_states,1); %initial state
initx(4) = z(1,:)'
% [xfilt, Vfilt] = KalmanFilter(z, F, H, Q, R, initx, initV);
x_estimate = KalmanFilter(z, F, H, Q, R, initx, P0);
%grabbing the data
estimate_data = zeros(length(x_estimate),n); 

for ii = 1:length(x_estimate)
    estimate_data(ii,:) = x_estimate{ii}(:)'; 
end 

%%%%%%%%%%%% Plotting

figure
set(gcf, 'color', 'k')
plot(1:length(x_estimate), z(:,1),'Color', 'r');hold on
plot(1:length(x_estimate), estimate_data(:,1),'Color', 'b') 
legend({'Measured','Kalman Estimate'},'Location','Southeast')
set(gca,'color','k')
set(gca,'ycolor','r')
set(gca,'xcolor','r')

%%
function state = KalmanFilter(z, F, H, Q, R, X0, P0)
R_1 = @(psi)[cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
R_2 = @(theta) [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R_3 = @(phi) [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
R_b__n = @(psi , theta , phi) R_1(psi)*R_2(theta)*R_3(phi);
global delta_t
    %state init
     X_star = cell(size(X0,1),1);
     P_star = cell(size(P0,1),1);
     X = cell(size(X0,1),1);
     P = cell(size(P0,1),1); 
for k = 1:size(z,1)
    %Future Projection
    if k == 1
        X_star{k} = F*X0; 
        P_star{k} = F*P0*F' + Q;
    else
       
        %Rbn = R_b__n(X{k-1}(7),X{k-1}(8),X{k-1}(9));
        %F(7:9 , 10:12) = Rbn*delta_t;

        X_star{k} = F*X_star{k-1};
        P_star{k} = F*P{k-1}*F' + Q;
        
    end 
    
    %measurement update
    %update gain
    K = P_star{k}*H'*inv(H*P_star{k}*H' + R); 
    X{k} = X_star{k} + K*(z(k,:)' - H*X_star{k}); 
    P{k} = (eye(size(X0)) - K*H)*P_star{k};
    
end 
state = X; 
end 
