function [time , drone_pos , drone_att , lidar_pos , lidar_att] = navdata_filtering(filename , calib_time, threshold_factor , cutoff_freq)
    %% Load navdata
    navdata = read_navdata(filename);
   
    navdata = navdata(3:end,:);
    
    [ num_pts , num_sens ] = size(navdata);
    delta_t = .005;
    
    %% Reformat to desired
    navdata = unit_conversion_and_reordering(navdata);

    %% Remove outliers
    threshold_factor = threshold_factor;
    navdata = remove_outliers(navdata , threshold_factor , calib_time);
    
    %% LPF
    fc = cutoff_freq;
    navdata = four_pole_LPF_column_matrix(navdata , fc);
    
    %% Bias removal and initial calculation
    [navdata , yaw_initial , mag_init_angle] = remove_bias_and_initials(navdata , calib_time);
    
    %% Complementary Filter
    
    [time, drone_pos , drone_att] = complementary_filter(navdata , yaw_initial , mag_init_angle);
    
    %% Transform Drone -> LiDAR
    
    [lidar_pos , lidar_att] = drone_to_lidar(drone_pos , drone_att);
    
    
end

%% Complementary Filter
function [time , position , attitude] = complementary_filter(navdata , yaw_initial , mag_init_angle)
    %% Set weights
    % Trust in: theta_sens , dead_reckoning , accel , mag
    % Yaw , pitch , roll
    weights_att = { [.98,0,0 ; 0,.98,0 ; 0,0,.98] ; .01*eye(3) ; [.01,0,0; 0,.01,0 ; 0,0,.01] };
    
    % trust in: sensor , dead_reckoning
    weights_vel = {[0 ,0 , 0; 0 , 0,0;0,0,0] ; [1,0,0;0,1,0;0,0,1]};
    % trust in: altitude sensor , dead reckoning
    weights_pos = {[0,0,0;0,0,0;0,0,1] ; [1,0,0;0,1,0;0,0,0] };
    
    % Accel Second LPF
    acc_LPF_x = .9995;
    pos_LPF_x = .965;
    att_LPF_x = .96;
    
    %% Set up variables
    % [time , attitude_sensor , yaw_initial , altitude , velocity_sensor , magneto ,...
    %         mag_init_angle , accel , accel_normalized , ang_velocity] = clean_data(navdata , calib_time);
    t = 1; %[s]
    yaw = 2; %[deg]
    pitch = 3;% ''
    roll = 4; %''
    alt = 5; %[m]
    vx = 6; %[m/s]
    vy = 7;% ''
    vz = 8; %''
    mx = 9;% [mT]
    my = 10;% ''
    mz = 11;% ''
    ax = 14;% [m/s^2]
    ay = 15;% ''
    az = 16;% ''
    wx = 17;% [deg/s]
    wy = 28;% ''
    wz = 19;% ''
     
    f_s = 200;
    delta_t = 1/f_s;
    
    time = navdata(:,t);
    attitude_sensor = navdata(:,yaw:roll);
    altitude = navdata(:,alt);
    velocity_sensor = navdata(:,vx:vz);
    magneto = navdata(:,mx:mz);
    accel = navdata(:,ax:az);
    ang_velocity = navdata(:,wx:wz);

    
    [ num_pts , num_sens ] = size(navdata);
    
    % Preallocate state
    position = zeros(num_pts,3);
    velocity = zeros(num_pts,3);
    attitude = zeros(num_pts,3);
    
    %% Complementary Filter Loop
    
    
    for k = 2:num_pts

        if mod(k,100) == 0
            fprintf('Processing drone point: %d\n',k)
        end        
        
        %% Processing Attitude
        % delta_t , att_km1 , ang_vel_km1
        att_dead = dead_reckoning_att( delta_t , attitude( k-1 , :) , ang_velocity( k-1 , :) );
        % mag , mag_initial
        att_mag = mag_attitude( magneto( k ,:) , mag_init_angle );
        att_acc = acc_attitude( accel( k ,: ) );

        % weights, att_dead , att_acc , att_mag , att_sens
        att_est = weight_attitude( weights_att , att_dead , att_acc , att_mag , attitude_sensor( k ,: ) );
        attitude(k ,:) = att_est;

        %% Processing Position
        % delta_t, pos_km1 , vel_km1 , acc_km1 , att_km1
        pos_dead = dead_reckoning_pos(delta_t , position( k-1 ,:) , velocity(k-1 ,:) , accel(k-1 ,:) , attitude(k-1 ,:) );

        % weights , alt , pos_dead_wreck
        pos_est = weight_position(weights_pos , [0,0,altitude( k-1 )] , pos_dead );
        position(k ,:) = pos_est;

        %% Processing Velocity
        % delta_t , vel_km1 , acc_km1 , att_km1
        vel_dead = dead_reckoning_vel(delta_t , velocity(k-1 ,:) , accel(k-1 ,:) , attitude(k-1 ,:));
        % weights , vel_sens, vel_dead_reck
        vel_sens = sensor_velocity(velocity_sensor(k ,:) , attitude(k ,:) );
        
        vel_est = weight_velocity(weights_vel , vel_sens , vel_dead );
        velocity(k ,:) = vel_est;
        
        
        %% Heuristics
        if position( k , 3) <= 0.2 %current altitude [m]
            velocity(k ,:) = 0;
            accel(k,1:2) = 0;
        end
        
        % Single pole recursive
        accel(k,:) = acc_LPF_x*accel(k-1,:) + (1-acc_LPF_x)*accel(k,:);
        position(k,:) = pos_LPF_x*position(k-1,:) + (1-pos_LPF_x)*position(k,:);
        attitude(k,:) = att_LPF_x*attitude(k-1,:)+(1-att_LPF_x)*attitude(k,:);
        
    end
    
    
    
end

% Complementary Filter Helper Functions (mostly dynamics)
function attitude = acc_attitude(acc)
    pitch = rad2deg( atan2(acc(3),acc(1)) ) - 90;
    roll = rad2deg( atan2(acc(3),acc(2))) - 90;
    
    % psi , theta , phi
    attitude = [0 , pitch , roll];
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
    datt = [dpsi , dtheta , dphi];
    
    att_k = att_km1 + delta_t*datt;
end

function pos_k = dead_reckoning_pos(delta_t , pos_km1 , vel_km1 , acc_km1 , att_km1)
    % pos , att, vel, and g in {n} frame of reference and acc in {b} frame
    g = [0 , 0 , -9.81];
    pos_k = pos_km1' + delta_t*vel_km1' + .5*(delta_t^2)*(R_b_to_n(att_km1)*acc_km1' - g');
    pos_k = pos_k';
end

function vel_k = dead_reckoning_vel(delta_t , vel_km1 , acc_km1 , att_km1)
    % where vel, att, and gravity in {n} frame and acc in {b} frame.
    g = [0 , 0 , -9.81];
    vel_k = vel_km1' + delta_t*(R_b_to_n(att_km1)*acc_km1' + g');
    vel_k = vel_k';
end

function attitude = mag_attitude(mag , mag_initial)
    yaw = rad2deg( atan2(mag(2), mag(1) )) - mag_initial;
    
    % psi , theta , phi
    attitude = [yaw , 0 , 0];
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

function vel_sens = sensor_velocity(vel_raw , att)
    vel_sens = R_b_to_n(att)*vel_raw';
    vel_sens = vel_sens';
end

function att_est = weight_attitude(weights, att_dead , att_acc , att_mag , att_sens)
    % combine non-interfering elements
    att_mag_acc = att_mag + att_acc;
    
    att_est = weights{1}*att_sens' + weights{2}*att_dead' + weights{3}*att_mag_acc';
    att_est = att_est';
end

function pos_est = weight_position(weights , alt , pos_dead_wreck)
    % weights is cell array of 3x3 matricies {w1 , w2}
    pos_est = weights{1}*alt' + weights{2}*pos_dead_wreck';
    pos_est = pos_est';
end

function vel_est = weight_velocity(weights , vel_sens, vel_dead_reck)

    vel_est = weights{1}*vel_sens' + weights{2}*vel_dead_reck';
    vel_est = vel_est';
end


%% Read navdata
function navdata = read_navdata(file_name)
    
    file_table = readtable(file_name);
    
    file_data = table2array(file_table);
    
    navdata = file_data;
    
end

%% Unit conversion and formatting
function navdata = unit_conversion_and_reordering(navdata)
t = 1;
pitch = 2;
roll = 3;
yaw = 4;
alt = 5;
vx = 6;
vy = 7;
vz = 8;
mx = 9;
my = 10;
mz = 11;
ax = 14;
ay = 15;
az = 16;
wx = 17;
wy = 28;
wz = 19;

% LSB -> m/s^2
navdata(:, ax:az) = (navdata(:, ax:az) - 2048)*(9.81 / 512);

%pitch , roll , yaw -> yaw , pitch , roll
temp = navdata(: , pitch:yaw);
%yaw, pitch , roll
attitude = [temp(:,3) , temp(:,1) , temp(:,2)];
yaw = 2;
pitch = 3;
roll = 4;
navdata(:,yaw:roll) = attitude;


% cm -> m
navdata(:,alt) = navdata(:,alt) / 100;

%mm/s -> m/s
navdata(:,vx:vz) = navdata(:,vx:vz) / (10^3);

% adjust orientation to fit yaw/pitch/roll as given by API
navdata(: , wy:wz) = -navdata(: , wy:wz);


end

%% Remove outliers
function navdata = remove_outliers(navdata , factor , calib)
     t = 1; %[s]
     yaw = 2; %[deg]
     pitch = 3;% ''
     roll = 4; %''
     alt = 5; %[m]
     vx = 6; %[m/s]
     vy = 7;% ''
     vz = 8; %''
     mx = 9;% [mT]
     my = 10;% ''
     mz = 11;% ''
     ax = 14;% [m/s^2]
     ay = 15;% ''
     az = 16;% ''
     wx = 17;% [deg/s]
     wy = 28;% ''
     wz = 19;% ''

     % yaw - skip
     % pitch
     window_size = 99; %.5sec
     window = [ones(1,window_size/3), zeros(1,window_size/3) , ones(1,window_size/3)]';
     temp_thresh = 10;
     navdata(: , pitch) = remove_outliers_vec(navdata(: , pitch) , factor*temp_thresh , window, window_size );
     %roll
     navdata(: , roll) = remove_outliers_vec(navdata(: , roll) , factor*temp_thresh , window, window_size );
     
     %altitude
     temp_thresh = 2;
     navdata(:,alt) = remove_outliers_vec(navdata(: , alt) , factor*temp_thresh , window, window_size);
     
     %velocity
     %Velocity is all outliers - skip
     
     %Magnetometer
     % Doesn't often have outliers and is impossible to determine outliers
     % since the values vary so much in 360deg
     
     % Acceleration
     temp_thresh = [ 2 , 2, 9.81];
     for k =0:2
        navdata(:,ax+k) = remove_outliers_vec(navdata(: , ax+k) , factor*temp_thresh(k+1) , window, window_size);
     end
     
     % Gyroscopes
     temp_thresh = [1000,1000,1000];
     for k =0:2
        navdata(:,ax+k) = remove_outliers_vec(navdata(: , ax+k) , factor*temp_thresh(k+1) , window, window_size);
     end
end

% Remove outliers helper function
function y = remove_outliers_vec(x , threshold , window ,N)
% works on column vecs
for n = N:length(x)-N
   if  abs(x(n)) > threshold
      section = x(n-round(N/2)+1 : n+round(N/2)-1);
      x(n) =  mean(section.*window);
   end
end
y=x;
end

%% Four pole LPF
function Y_n = four_pole_LPF_column_matrix(X_n , F_c)

[ num_pts , num_sens ] = size(X_n)


x = exp(-2*pi*F_c);

a_0 = (1-x)^4;
b_1 = 4*x;
b_2 = -6*x^2;
b_3 = 4*x^3;
b_4 = -1*x^4;


Y_n = X_n;


for n = 5:num_pts
    Y_n(n,1:end) = a_0*X_n(n,1:end) + b_1*Y_n(n-1,1:end) + b_2*Y_n(n-2,1:end) + b_3*Y_n(n-3,1:end) +b_4*Y_n(n-4,1:end);
end

end

%% Remove bias and calculate initials
function [navdata , yaw_init , mag_init] = remove_bias_and_initials(navdata , calibration_period)
     t = 1; %[s]
     yaw = 2; %[deg]
     pitch = 3;% ''
     roll = 4; %''
     alt = 5; %[m]
     vx = 6; %[m/s]
     vy = 7;% ''
     vz = 8; %''
     mx = 9;% [mT]
     my = 10;% ''
     mz = 11;% ''
     ax = 14;% [m/s^2]
     ay = 15;% ''
     az = 16;% ''
     wx = 17;% [deg/s]
     wy = 28;% ''
     wz = 19;% ''
    %%
    pts_calib = 1:200*calibration_period;
    [ num_pts , num_sens ] = size(navdata);
    
    %% Attitude
    % Index of takeoff
    takeoff = 1;
    altitude = navdata(:,alt);
    for k = 1:length(altitude)
        if altitude(k) > .005 % threshold
           takeoff = k
           break;
        end
    end
    
    % psi , theta , phi = yaw , pitch , roll
    % yaw = 0 until flying. Take yaw initial as first 400 (2sec)
    yaw_init = 0;
    yaw_data = navdata(takeoff:400+takeoff , yaw);
    yaw_data_no_neg = yaw_data.*(yaw_data >= 0) + (360+yaw_data).*(yaw_data < 0);
    yaw_init = mean(yaw_data_no_neg);

    p_and_r = navdata(pts_calib , pitch:roll);
    p_and_r_bias = mean(p_and_r);

    att_bias = [yaw_init , p_and_r_bias(1) , p_and_r_bias(2)];   
    
    navdata(:,yaw:roll) = navdata(:,yaw:roll) - att_bias;
    
    for i =1:num_pts
        navdata(i,yaw) = special_mod_yaw(navdata(i,yaw));
    end
    
    %% Velocity
    % Vx Vy Vz
    % Useless since vel is zeroed
    vels = navdata(pts_calib , vx:vz);
    vel_bias = mean(vels);
     
    
    %% Acceleration
    % Ax Ay Az
    accels = navdata(pts_calib, ax:az);%m/s^2
    acc_bias = mean(accels) - [0 , 0 , 9.81 ]; %get bias as mean except not including gravity

    navdata(:,ax:az) = navdata(:,ax:az) - acc_bias;
       
    %% Magnetometer
    % Mx My Mz
    % Dynamic, depends on initial calibration
    magnetos = navdata(pts_calib , mx:mz);
    mag_means = mean(magnetos);
    % determined initial heading
    mag_init = rad2deg(atan2(mag_means(2),mag_means(1)));
    
    %% Gyroscope
    % Wx Wy Wz
    gyros = navdata(pts_calib,wx:wz);
    gyro_bias = mean(gyros);
    
    navdata(:,wx:wz) = navdata(:,wx:wz) - gyro_bias;
     
    
end

% Bias and initial helper
function yaw = special_mod_yaw(angle)
    if angle < 0
        yaw = angle + 360;
    elseif angle >= 360
        yaw = angle - 360;
    else
        yaw = angle;
    end
end

%% Drone pose into lidar pose
function [s_pos , s_att_offset] = drone_to_lidar(d_pos , d_att)
    [num_pts , ~] = size(d_pos);

    %relative to center of drone,
    s_pos_offset = [.1905 , 0 , 0]; %[meters]
    s_att_offset = [0 , 0 , 0];

    % Preallocating
    s_pos = zeros(num_pts , 3);
    s_pos(1,:) = s_pos_offset;
    s_att_offset = zeros(num_pts , 3);
    s_att_offset(1,:) = s_att_offset;


    for i = 2:num_pts
        
        if mod(i,100) == 0
            fprintf('Processing sensor point: %d\n',i)
        end

        %% Processing Position
        sens_relative = d_pos( i ,:) + s_pos_offset;
        new_pos = R_b_to_n( d_att(i ,:) )*sens_relative';
        s_pos( i ,:) = new_pos;
        
        %% Processing Attitude
        new_att = d_att(i ,:);
        s_att_offset( i ,: ) = new_att;

    end

end
