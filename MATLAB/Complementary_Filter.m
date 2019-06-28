%% Complementary Filter. Goal: All sensor data -> p , v , theta (attitude)
clear all;
close all;
clc;

file_1 = 'calibration_ar174_flying_data.txt';
file_2 = 'calibration_ar174_flying_2_data.txt';

file_1_table = readtable(file_1);
file_2_table = readtable(file_2);

%time[1] : pitch[2] : roll[3] : yaw[4] : alt(demo)[5] : ...
%   Vx[6] : Vy[7] :Vz[8] : Mx[9] : My[10] : Mz[11] : ...
%   alt(vision)[12] : alt(raw)[13] : Ax[14] : Ay[15] : Az[16] ...
%   Wx[17] : Wy[18] : Wz[19]
file_1_data = table2array(file_1_table);
file_2_data = table2array(file_2_table);

total = [ file_1_data ; file_2_data ];

[ num_pts , num_sens ] = size(total);

delta_t = total(2,1)-total(1,1);

%% Anonymous Functions

R_1 = @(psi)[cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
R_2 = @(theta) [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R_3 = @(phi) [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
%R_b__n = @(psi , theta , phi) R_1(psi)*R_2(theta)*R_3(phi);
R_b__n = @(theta_vec) R_1(theta_vec(3))*R_2(theta_vec(1))*R_3(theta_vec(2));

s = @(angle) (sin(deg2rad(angle)));
c = @(angle) (cos(deg2rad(angle)));
t = @(angle) (tan(deg2rad(angle)));
C_b__n = @(theta) ([0 , c(theta(1)) , -s(theta(2)) ; 1 , s(theta(2))*t(theta(1)) , c(theta(2))*t(theta(1)) ; 0 , s(theta(2))/c(theta(1)) , c(theta(2))/c(theta(1)) ]);

pos_kp1 = @( delta_t , p_n , v , a , theta_vec ) (p_n + delta_t*R_b__n(theta_vec)*v + .5*(delta_t^2)*R_b__n(theta_vec)*a);

vel_kp1 = @(delta_t , v_n , a , theta_vec) (v_n + delta_t*R_b__n(theta_vec)*a);

theta_vec_kp1_dead_reck = @(delta_t , theta_vec_n , omega ) (theta_vec_n + delta_t*C_b__n(theta_vec_n)*omega);

theta_vec_k_acc = @(accel) ([ rad2deg( atan2(accel(3),accel(1)) ) ; rad2deg( atan2(accel(3),accel(2))) ; 0 ]);

theta_vec_k_mag = @(magnet , orig_mag) ([ 0 ; 0 ; rad2deg( atan2(orig_mag(2),orig_mag(1)) - atan2(magnet(2) ,magnet(1) ) ) ]);

% weights is (4x3)
comp_filt_theta_vec_kp1 = @(mag_orig , weights , delta_t , theta_sens_kp1 , theta_vec_k , omega_k , accel_kp1 , mag_kp1 ) ...
                        (weights{1}*theta_sens_kp1 + ...
                        weights{2}*theta_vec_kp1_dead_reck(delta_t , theta_vec_k , omega_k) + ...
                        weights{3}*theta_vec_k_acc(accel_kp1) + ...
                        weights{4}*theta_vec_k_mag(mag_kp1 , mag_orig));

% weights is (2x3)
comp_filt_vel_kp1 = @(weights , vel_sens_kp1 , delta_t , vel_k , accel_k , theta_vec_k) ...
                  (weights{1}*vel_sens_kp1 + ...
                  weights{2}*vel_kp1(delta_t , vel_k , accel_k , theta_vec_k));
                  

             
                  
%% Cleaning Data  
%time[1] : pitch[2] : roll[3] : yaw[4] : alt(demo)[5] : ...
%   Vx[6] : Vy[7] :Vz[8] : Mx[9] : My[10] : Mz[11] : ...
%   alt(vision)[12] : alt(raw)[13] : Ax[14] : Ay[15] : Az[16] ...
%   Wx[17] : Wy[18] : Wz[19]

means = mean(total);
stds = std(total);
variances = var(total);

% Attitude: Find mean of attitude (adjusted for >180 is negative)
att_data =total( : , 2:4);
att_data_adjusted = att_data.*(att_data >= 0) + (180-att_data.*(att_data < 0 ));
att_mean = mean(att_data_adjusted);
att_mean = att_mean.*(att_mean < 180) + (att_mean.*(att_mean > 180) - 180);
means(2:4) = att_mean;

maybe_cleaned_theta_sens = total(:,2:4) - att_mean;
maybe_cleaned_theta_sens = maybe_cleaned_theta_sens.*(maybe_cleaned_theta_sens > -180) + ((360+ maybe_cleaned_theta_sens).*(maybe_cleaned_theta_sens<-180));


% Acceleration: convert to G 
acc_data = (total( : , 14:16 )-2048)/512;
acc_data(:,3) = acc_data(:,3)*-1;


% Acceleration: and normalize
for i =1:num_pts
   acc_data(i,:) = acc_data(i,:)/norm(acc_data(i,:));
end
accel_sens = acc_data;

% Velocity:
vel_sens = total(: , 6:8) - means(6:8);


% Magnetometer:
mag_data = total(: ,9:11);
mag_orig = mean(mag_data(1:1000 , :));
mag_sens = total(:,9:11);

% Gyroscope:
gyro_data = total(:,17:19);
gyro_sens = gyro_data - - means(17:19);


%% Filtering Data

% STATE VARIABLES
%
position = [ 0 ; 0 ; 0 ];
velocity = [ 0 ; 0 ; 0 ];
theta_vec = [0;0;0];
%

%time[1] : pitch[2] : roll[3] : yaw[4] : alt(demo)[5] : ...
%   Vx[6] : Vy[7] :Vz[8] : Mx[9] : My[10] : Mz[11] : ...
%   alt(vision)[12] : alt(raw)[13] : Ax[14] : Ay[15] : Az[16] ...
%   Wx[17] : Wy[18] : Wz[19]

% Trust in: theta_sens , dead_reckoning , accel/mag
weights_theta = { .5*eye(3) ; .1*eye(3) ; .4*[1 , 0 ,0 ;0,1,0;0,0,0] ; .4*[0,0,0;0,0,0;0,0,1]};
weights_vel = {.8*eye(3) ; .2*eye(3)};

figure;

for i = 2:num_pts

    pos_est = pos_kp1( delta_t , position(:,i-1) , vel_sens(i-1,:)' , accel_sens(i-1,:)' , theta_vec(:,i-1) );
    position = [position , pos_est];
    for p = 1:3
        subplot(3,3,p);
        plot(i,pos_est(p),'*');
        hold on;
    end
        
    vel_est = comp_filt_vel_kp1(weights_vel , vel_sens(i,:)' , delta_t , vel_sens(i-1,:)' , accel_sens(i,:)' , theta_vec(:,i-1) );
    velocity = [velocity , vel_est ];
    for p = 1:3
        subplot(3,3,p+3);
        plot(i,vel_est(p),'*');
        hold on;
    end

    % @(mag_orig , weights , delta_t , theta_sens_kp1 , theta_vec_k , omega_k , accel_kp1 , mag_kp1 )
    att_est = comp_filt_theta_vec_kp1(mag_orig , weights_theta , delta_t , maybe_cleaned_theta_sens(i ,:)' , theta_vec(: , i-1) , ...
                gyro_sens(i,:)' , accel_sens(i ,:)' , mag_sens(i,:)');
    theta_vec = [theta_vec , att_est ]; 
    for p = 1:3
        subplot(3,3,p+6);
        plot(i,att_est(p),'*');
        hold on;
    end
     
    pause(.1);
end

%%%%%%%%%%  YOU FORGOT ALTITUDE %%%%%%%%%%%%%%%%%%%%
