%% Complementary Filter. Goal: All sensor data -> p , v , theta (attitude)
clear all;
close all;
clc;

nav_data = read_navdata();

%offset = 10000

% [ num_pts , num_sens ] = size(nav_data);
% 
% nav_data = nav_data(offset:num_pts , :);

[ num_pts , num_sens ] = size(nav_data);

delta_t = nav_data(2,1)-nav_data(1,1);

% column vecs
[attitude_sensor , yaw_initial , altitude , velocity_sensor , magneto , mag_init_angle , accel_no_grav , accel_normalized , ang_velocity] = clean_data(nav_data);

%% Filtering Data

num_to_filter = num_pts

% STATE VARIABLES 
% Initials
position = [ 0 ; 0 ; 0 ];
velocity = [ 0 ; 0 ; 0 ];
attitude = [ 0 ; 0 ; 0 ];

% Preallocate
position = zeros(3,num_to_filter);
velocity = zeros(3,num_to_filter);
attitude = zeros(3,num_to_filter);

%time[1] : pitch[2] : roll[3] : yaw[4] : alt(demo)[5] : ...
%   Vx[6] : Vy[7] :Vz[8] : Mx[9] : My[10] : Mz[11] : ...
%   alt(vision)[12] : alt(raw)[13] : Ax[14] : Ay[15] : Az[16] ...
%   Wx[17] : Wy[18] : Wz[19]

% Trust in: theta_sens , dead_reckoning , accel/mag
weights_att = { .6*eye(3) ; .3*eye(3) ; .1*eye(3)};
% trust in: sensor , dead_reckoning
weights_vel = {[.3 ,0 , 0; 0 , .7,0;0,0,0] ; [.7,0,0;0,.3,0;0,0,1]};
% trust in: alt sens , dead reckoning
weights_pos = {[0,0,0;0,0,0;0,0,.9] ; [1,0,0;0,1,0;0,0,.1] };

%figure;

for k = 2:num_to_filter
    
    if mod(k,100) == 0
        fprintf('Processing point: %d\n',k)
    end
    
     % delta_t , att_km1 , ang_vel_km1
    att_dead = dead_reckoning_att( delta_t , attitude(: , k-1) , ang_velocity(:, k-1) );

    % mag , mag_initial
    att_mag = mag_attitude( magneto(: , k) , mag_init_angle );

    att_acc = acc_attitude( accel_normalized( : , k ) );

    % weights, att_dead , att_acc , att_mag , att_sens
    att_est = weight_attitude( weights_att , att_dead , att_acc , att_mag , attitude_sensor( : , k ) );
    
    attitude(: , k) = att_est;
    %attitude = [attitude , att_est ]; 
%     for p = 1:3
%         subplot(3,3,p+6);
%         plot(k,att_est(p),'*');
%         hold on;
%     end
     
    %drawnow;
    %pause(0.00001);
    
    
    % delta_t, pos_km1 , vel_km1 , acc_km1 , att_km1
    pos_dead = dead_reckoning_pos(delta_t , position(: , k-1) , velocity(: , k-1) , accel_no_grav(: , k-1) , attitude(: , k-1) );
    
    % weights , alt , pos_dead_wreck
    pos_est = weight_position(weights_pos , [0;0;altitude( k-1 )] , pos_dead );
    
    position(:,k) = pos_est;
    
    %position = [position , pos_est];
%     for p = 1:3
%         subplot(3,3,p);
%         plot(k,pos_est(p),'*');
%         hold on;
%     end

    % delta_t , vel_km1 , acc_km1 , att_km1
    vel_dead = dead_reckoning_vel(delta_t , velocity(: , k-1) , accel_no_grav(:,k-1) , attitude(: , k-1));
    % weights , vel_sens, vel_dead_reck
    vel_sens = sensor_velocity(velocity_sensor(: , k) , attitude(: , k) );
    vel_est = weight_velocity(weights_vel , vel_sens , vel_dead );
    
    velocity(:, k) = vel_est;
    
    %velocity = [velocity , vel_est ];
%     for p = 1:3
%         subplot(3,3,p+3);
%         plot(k,vel_est(p),'*');
%         hold on;
%     end
end

close all;
figure;

for p = 1:3
         subplot(3,3,p);
         plot(1:num_to_filter,position(p,:),'-');
         hold on;
end

for p = 1:3
         subplot(3,3,p+3);
         plot(1:num_to_filter,velocity(p,:),'-');
         hold on;
end
     
for p = 1:3
         subplot(3,3,p+6);
         plot(1:num_to_filter,attitude(p,:),'-');
         hold on;
end

%% Calculate sensor pose
%%%%% Broken at the moment

%relative to center of drone,
sensor_pos = [.1905 ; 0 ; 0]; %meters
sensor_att = [0 ; 0 ; 0];

for i = 1:num_to_filter
   new_pos(: , i) = R_b_to_n( attitude(: , i) )*sensor_pos( :, i) + position( :, i);
   sensor_pos = [sensor_pos , new_pos]; 
   new_att(: ,i) = attitude(: , i);
   sensor_att = [sensor_att , new_att];
    
end

figure;

for p = 1:3
         subplot(3,1,p);
         plot(1:num_to_filter,sensor_pos(p,:),'-');
         hold on;
end

