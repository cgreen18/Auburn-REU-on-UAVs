clear; clc; close all; 
%estimate scalar random constant voltage reading from a source
figure
set(gcf, 'color', 'k')

%indirectly measured values
rng('shuffle','twister')
z = (.5-.39).*rand(1000,1) + 0.39; 
%initial state
X0 = 0; 
%intital error covariance
P0 = 1; 

% the state transition matrix is 1 and the measurement transition matrix is
% H is 1
A = 1; H = 1; 
%also assuming process noise covariance Q = 0 and measurement noise
%covariance 
R = 0.1
Q = 0; 
output = KalmanFilter(X0,P0, z, A, H, R, Q);
plot(1:length(z),output,'Linewidth',3)
set(gca,'color','k')
set(gca,'ycolor','r')
set(gca,'xcolor','r')


%% k = 1 First discrete time step
%Time update
X1 = A.*X0
P1 = A.*P0.*A'

%Measurement update
%Kalman Gain
K1 = P1.*transpose(H).*inv(H.*P1.*transpose(H) + R)
%most current measurement update
XK = X1 + K1.*(z(1) - H.*X1)
%update covariance error 
P1 = (eye(1) - K1.*H).*P1

%%
function finalState = KalmanFilter(X0, P0, Z, A, H, R, Q)
    %state init
     X = zeros(length(Z),1);
     P = zeros(length(Z),1); 
for k = 1:length(Z)
    %Future Projection
    if k == 1
        X = A.*X0; P = A.*P0.*A' + Q;
    else
        X(k) = A.*X(k - 1);
        P(k) = A.*P(k-1).*A' + Q;
    end 
    
    %measurement update
    %update gain
    K = P(k).*H'.*inv(H.*P(k).*H' + R); 
    X(k) = X(k) + K.*(Z(k) - X(k)); 
    P(k) = (eye(1) - K).*P(k); 
 
end 
finalState = X; 
end 