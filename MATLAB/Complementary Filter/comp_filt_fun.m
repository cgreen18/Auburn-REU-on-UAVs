%% Complementary Filter. Goal: All sensor data -> p , v , theta (attitude)
% Complementary Filter function - comp_filt_fun(name of file) -> drone...
% position, velocity, and attitude & sensor position and attitude

function [time , d_pos , d_vel , d_att , s_pos , s_att] = comp_filt_fun(filename , calib_time)

    %% Read in file
    nav_data = read_navdata(filename);
   
    nav_data = nav_data(3:end,:);
    
    [ num_pts , num_sens ] = size(nav_data);
    delta_t = .005;
    
    [time , attitude_sensor , yaw_initial , altitude , velocity_sensor , magneto ,...
        mag_init_angle , accel , accel_normalized , ang_velocity] = clean_data(nav_data , calib_time);
    
    plot_3_plots_row(accel);
   

    accel = movmean(accel, 101 , 2);
    accel = movmean(accel, 201 , 2);
    %accel = movmean(accel, 301 , 2);
    

    plot_3_plots_row(accel);
    
    %% Filter Drone Pose
    
    % Can be modified for debugging
    num_to_filter = num_pts


    % STATE VARIABLES 
    % Initials
%     % X Y Z
%     position = [ 0 ; 0 ; 0 ];
%     % Vx Vy Vz
%     velocity = [ 0 ; 0 ; 0 ];
%     % psi theta phi
%     attitude = [ 0 ; 0 ; 0 ];

    % Preallocate
    position = zeros(3,num_to_filter);
    velocity = zeros(3,num_to_filter);
    attitude = zeros(3,num_to_filter);

    % Trust in: theta_sens , dead_reckoning , accel , mag
    % Yaw , pitch , roll
     weights_att = { [.7,0,0 ; 0,.6,0 ; 0,0,.6] ; .05*eye(3) ; [0,0,0; 0,.3,0 ; 0,0,.3] ; [.25 , 0 , 0 ; 0,0,0 ; 0,0,0]};
    %weights_att = { [.8,0,0 ; 0,.85,0 ; 0,0,.85] ; 0*eye(3) ; [0,0,0; 0,.15,0 ; 0,0,.15] ; [.2 , 0 , 0 ; 0,0,0 ; 0,0,0]};
    weights_att = { [0,0,0 ; 0,.6,0 ; 0,0,.6] ; 0*eye(3) ; [1,0,0; 0,.3,0 ; 0,0,.3] };
    
    % trust in: sensor , dead_reckoning
    weights_vel = {[0 ,0 , 0; 0 , 0,0;0,0,0] ; [1,0,0;0,1,0;0,0,1]};
    % trust in: alt sens , dead reckoning
    weights_pos = {[0,0,0;0,0,0;0,0,.95] ; [1,0,0;0,1,0;0,0,.05] };
    
    % smooothing km1 vs k weight
    smooth_pos = .5
    smooth_vel = .2
    smooth_acc = .99
    smooth_att = 0

    for k = 2:num_to_filter

        if mod(k,100) == 0
            fprintf('Processing drone point: %d\n',k)
        end
        
        if k==7000
            fprintf('hep for debug')
        end
    
        %velocity_sensor(: , k) = .2*velocity_sensor(: , k);
        
        
        %% Processing Attitude
        % delta_t , att_km1 , ang_vel_km1
        att_dead = dead_reckoning_att( delta_t , attitude(: , k-1) , ang_velocity(:, k-1) );
        % mag , mag_initial
        att_mag = mag_attitude( magneto(: , k) , mag_init_angle );
        att_acc = acc_attitude( accel_normalized( : , k ) );

        % weights, att_dead , att_acc , att_mag , att_sens
        att_est = weight_attitude( weights_att , att_dead , att_acc , att_mag , attitude_sensor( : , k ) );
        attitude(: , k) = att_est;

        %% Processing Position
        % delta_t, pos_km1 , vel_km1 , acc_km1 , att_km1
        pos_dead = dead_reckoning_pos(delta_t , position(: , k-1) , velocity(: , k-1) , accel(: , k-1) , attitude(: , k-1) );

        % weights , alt , pos_dead_wreck
        pos_est = weight_position(weights_pos , [0;0;altitude( k-1 )] , pos_dead );
        position(:,k) = pos_est;

        %% Processing Velocity
        % delta_t , vel_km1 , acc_km1 , att_km1
        vel_dead = dead_reckoning_vel(delta_t , velocity(: , k-1) , accel(:,k-1) , attitude(: , k-1));
        % weights , vel_sens, vel_dead_reck
        vel_sens = sensor_velocity(velocity_sensor(: , k) , attitude(: , k) );
        
        vel_est = weight_velocity(weights_vel , vel_sens , vel_dead );
        velocity(:, k) = vel_est;
        
        %% Smoothing
        position(:,k) = smooth_pos*position(:,k-1) + (1-smooth_pos)*position(:,k);
        velocity(: , k) = smooth_vel*velocity(:,k-1) + (1-smooth_vel)*velocity(:,k);
        accel(: , k) = smooth_acc*accel(: , k-1) + (1-smooth_acc)*accel(:,k);
        attitude(:,k)= smooth_att*attitude(:,k-1) + (1-smooth_att)*attitude(:,k);
        
        % Helpful cheat
        if position( 3 , k) <= .2
            velocity(:,k) = 0;
        end
        
        

    end   
    
    plot_3_plots_row(accel);
    
    %% Calculate Sensor Pose

    %relative to center of drone,
    sensor_offset = [.1905 ; 0 ; 0]; %[meters]
    sensor_att = [0 ; 0 ; 0];

    % Preallocating
    sensor_pos = zeros(3,num_to_filter);
    sensor_att = zeros(3,num_to_filter);


    for i = 2:num_to_filter
        
        if mod(i,100) == 0
            fprintf('Processing sensor point: %d\n',i)
        end

        %% Processing Position
        sens_relative = position(: , i) + sensor_offset;
        new_pos = R_b_to_n( attitude(: , i) )*sens_relative;
        sensor_pos( : , i) = new_pos;
        
        %% Processing Attitude
        new_att = attitude(: , i);
        sensor_att( :, i ) = new_att;

    end

    %% Format for output
    time = time';
    
    d_pos = position';
    d_vel = velocity';
    d_att = attitude';
    
    s_pos = sensor_pos';
    s_att = sensor_att';
    
    
end

% returns all these as COLUMN vectors
function [time , attitude , yaw_initial , altitude , velocity , magneto , mag_initial , accel , accel_normalized , ang_velocity] = clean_data(nav_data , calibration_period)
% total_mat format:
% time[1] : pitch[2] : roll[3] : yaw[4] : alt(demo)[5] : ...
%   Vx[6] : Vy[7] :Vz[8] : Mx[9] : My[10] : Mz[11] : ...
%   alt(vision)[12] : alt(raw)[13] : Ax[14] : Ay[15] : Az[16] ...
%   Wx[17] : Wy[18] : Wz[19]

t = 1
pitch = 2
roll = 3
yaw = 4
alt = 5
vx = 6
vy = 7
vz = 8
mx = 9
my = 10
mz = 11
ax = 14
ay = 15
az = 16
wx = 17
wy = 28
wz = 19


%% Create unique variables
% t = 1
% pitch = 2
% roll = 3
% yaw = 4
% alt = 5
% vx = 6
% vy = 7
% vz = 8
% mx = 9
% my = 10
% mz = 11
% ax = 14
% ay = 15
% az = 16
% wx = 17
% wy = 28
% wz = 19

time = nav_data(:,t);
temp = nav_data(: , pitch:yaw);
attitude = [temp(:,3) , temp(:,1) , temp(:,2)];
altitude = nav_data(:,alt);
velocity = nav_data(: , vx:vz);
magneto = nav_data(: , mx:my);
accel = (nav_data(:, ax:az) - 2048) *(1000/ 512); %mG
ang_velocity = nav_data(:, wx:wz);


%% Calibration
pts_calib = 1:200*calibration_period;

% Vx Vy Vz
vels = nav_data(pts_calib , vx:vz);
vel_bias = mean(vels);


% Mx My Mz
% Dynamic, depends on initial calibration
magnetos = nav_data(pts_calib , mx:mz);
mag_means = mean(magnetos);

% Ax Ay Az
accels = (nav_data(pts_calib, ax:az) - 2048)*(1000/512); %mG
acc_bias = mean(accels) - [0 , 0 , 1000 ]; %get bias as mean except not including gravity

% Wx Wy Wz
gyros = nav_data(pts_calib,wx:wz);
gyro_bias = mean(gyros);




%% Subtracting bias & Converting to nice coordinate system
% t = 1
% pitch = 2
% roll = 3
% yaw = 4
% alt = 5
% vx = 6
% vy = 7
% vz = 8
% mx = 9
% my = 10
% mz = 11
% ax = 14
% ay = 15
% az = 16
% wx = 17
% wy = 28
% wz = 19

function yaw = special_mod_yaw(angle)
    if angle < 0
        yaw = angle + 360;
    elseif angle >= 360
        yaw = angle - 360;
    else
        yaw = angle;
    end
end


% Index of takeoff
takeoff = 0;
for k = 1:length(altitude)
    if altitude(k) > .2
       takeoff = k
       break;
    end
end
if takeoff == 0
   takeoff = calibration_period*200;
end


% psi , theta , phi = yaw , pitch , roll
% yaw = 0 until flying. Take yaw initial as first 200 (1sec)
yaw_data = nav_data(takeoff:800+takeoff , yaw);
yaw_data_no_neg = yaw_data.*(yaw_data >= 0) + (360+yaw_data).*(yaw_data < 0);
yaw_initial = mean(yaw_data_no_neg);
yaw_initial = 0;

p_and_r = nav_data(pts_calib , pitch:roll);
p_and_r_bias = mean(p_and_r);

att_bias = [yaw_initial , p_and_r_bias(1) , p_and_r_bias(2)];


% psi , theta , phi
attitude = attitude - att_bias;
for i =1:length(attitude)
    attitude(i,1) = special_mod_yaw(attitude(i,1));
end

% subtract bias
velocity = velocity - vel_bias;

% normalize magneto by row
magneto = normr(magneto);

% 
% for ii = 1:size(magneto,1)
%     magneto(ii,:) = magneto(ii,:)/norm(magneto(ii,:));
% end


% determined initial heading
mag_initial = rad2deg(atan2(mag_means(2),mag_means(1)));

% Subtract bias
% flip all axes
accel = accel - acc_bias;
%accel = -accel;
%accel(1) = -accel(1);


% for direction calcs

accel_normalized = normr(accel);

% for ii = 1:size(accel,1)
%     accel_normalized(ii,:) = accel(ii,:)/norm(accel(ii,:));
% end 


% subtract bias
% adjust orientation to fit yaw/pitch/roll as given by API
ang_velocity = ang_velocity - gyro_bias;
ang_velocity(:,2) = -ang_velocity(:,2);
ang_velocity(:,3) = -ang_velocity(:,3);

%% Convert units
% cm -> m
altitude = altitude / 100;

%mm/s -> m/s
velocity = velocity / 10^3;

% mG -> m/s^2
accel = accel*9.81/10^3;

%% make row vecs
time = time';
attitude = attitude';
altitude = altitude';
velocity = velocity';
magneto = magneto';
accel = accel';
accel_normalized = accel_normalized';
ang_velocity = ang_velocity';

end

function navdata = read_navdata(file_name)
    
    file_table = readtable(file_name);
    
    file_data = table2array(file_table);
    
    navdata = file_data;
    
end

function att_k = dead_reckoning_att(delta_t , att_km1 , ang_vel_km1)
    % attitude in {n} and angular velocity in {b}
    
    
%     R_z = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
%     R_y = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
%     R_x = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    
    %convenience
    theta = att_km1(2);
    phi = att_km1(3);
    psi = att_km1(1);
    omega = ang_vel_km1;
    
    c = @(angle) (cos(deg2rad(angle)));
    s = @(angle) (sin(deg2rad(angle)));
    t = @(angle) (tan(deg2rad(angle)));
    
    dtheta = omega(2)*c(theta)-omega(3)*s(phi);
    dphi = omega(1) + omega(2)*s(phi)*t(theta) + omega(3)*c(phi)*t(theta);
    dpsi = omega(2)*(s(phi)/c(theta)) + omega(3)*(c(phi)/c(theta));
    
    % psi , theta , phi
    datt = [dpsi ; dtheta ; dphi];
    
    att_k = att_km1 + delta_t*datt;
end

function attitude = mag_attitude(mag , mag_initial)
    yaw = rad2deg( atan2(mag(2), mag(1) )) - mag_initial;
    
    % psi , theta , phi
    attitude = [yaw ; 0 ; 0];
end

function attitude = acc_attitude(acc)
    pitch = rad2deg( atan2(acc(3),acc(1)) ) -90;
    roll = rad2deg( atan2(acc(3),acc(2))) -90;
    
    % psi , theta , phi
    attitude = [0 ; pitch ; roll];
end

function pos_k = dead_reckoning_pos(delta_t , pos_km1 , vel_km1 , acc_km1 , att_km1)
    % pos , att, vel, and g in {n} frame of reference and acc in {b} frame
    g = [0 ; 0 ; -9.81];
    pos_k = pos_km1 + delta_t*vel_km1 + .5*(delta_t^2)*(R_b_to_n(att_km1)*acc_km1 - g);
end

function vel_k = dead_reckoning_vel(delta_t , vel_km1 , acc_km1 , att_km1)
    % where vel, att, and gravity in {n} frame and acc in {b} frame.
    g = [0 ; 0 ; -9.81];
    vel_k = vel_km1 + delta_t*(R_b_to_n(att_km1)*acc_km1 - g);
end

function vel_sens = sensor_velocity(vel_raw , att)
    vel_sens = R_b_to_n(att)*vel_raw;
end

function Rmat = R_b_to_n(euler_vec)
    % euler_vec is in order: psi , theta , phi

    euler_vec = deg2rad(euler_vec);
    psi = euler_vec(1);
    theta = euler_vec(2);
    phi = euler_vec(3);

    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    
    Rmat = R_z*R_y*R_x;
    % R_mat(3,1) = -sin(theta)
    % -> theta = -arcsin(R_mat(3,1))
    
    % R_mat(3,2)/R_mat(3,3) = tan(psi)
    % -> psi = atan2(R_mat(3,2),R_mat(3,3))
    % or to avoid error, psi = atan2(R32/cos(theta) , R33/cos(theta))
    
    % R_mat(2,1) / R_mat(1,1) = tan(phi)
    % phi = atan2(r21/cos(theta) , R11/cos(theta))
end

function Rmat = R_n_to_b(euler_vec)
    %or transpose of R_b_to_n but nice to have both explicitly written

    euler_vec = deg2rad(euler_vec);
    psi = euler_vec(1);
    theta = euler_vec(2);
    phi = euler_vec(3);

    R_z = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    R_y = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    R_x = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    
    Rmat = R_z*R_y*R_x;
    % R_mat(1,3) = -sin(theta)
end

function att_est = weight_attitude(weights, att_dead , att_acc , att_mag , att_sens)
    % combine non-interfering elements
    att_mag_acc = att_mag + att_acc;
    
    att_est = weights{1}*att_sens + weights{2}*att_dead + weights{3}*att_mag_acc;
end

function pos_est = weight_position(weights , alt , pos_dead_wreck)
    % weights is cell array of 3x3 matricies {w1 , w2}
    pos_est = weights{1}*alt + weights{2}*pos_dead_wreck;
end

function vel_est = weight_velocity(weights , vel_sens, vel_dead_reck)

    vel_est = weights{1}*vel_sens + weights{2}*vel_dead_reck;

end

function plot2D_drone_multiplot(d_pos, d_vel , d_att)

num_pts = length(d_pos);

figure;


pos_plot_names = {'X pos' , 'Y pos' , 'Z pos'};
vel_plot_names = {'V_x' , 'V_y' , 'V_z'};
att_plot_names = {'Yaw' , 'Pitch' , 'Roll'};

for p = 1:3
         subplot(3,3,p);
         plot(1:num_pts,d_pos(:,p),'-');
         title(pos_plot_names{p});
         xlabel("Time [samples]");
         ylabel("Distance [m]");
         hold on;
end

for p = 1:3
         subplot(3,3,p+3);
         plot(1:num_pts,d_vel(:,p),'-');
         title(vel_plot_names{p});
         xlabel("Time [samples]");
         ylabel("Speed [m/s]");
         hold on;
end
     
for p = 1:3
         subplot(3,3,p+6);
         plot(1:num_pts,d_att(:,p),'-');
         title(att_plot_names{p});
         xlabel("Time [samples]");
         ylabel("Angle [deg.]");
         hold on;
end

end

function write_to_file(name , time , d_pos , d_vel , d_att , s_pos , s_att)

total = [time ,d_pos , d_vel , d_att , s_pos , s_att];
total_table = array2table(total);
writetable(total_table, name , 'WriteVariableNames',false);

end
