% returns all these as COLUMN vectors
function [attitude , yaw_initial , altitude , velocity , magneto , mag_initial , accel_no_grav , accel_normalized , ang_velocity] = clean_data(nav_data)
% total_mat format:
% time[1] : pitch[2] : roll[3] : yaw[4] : alt(demo)[5] : ...
%   Vx[6] : Vy[7] :Vz[8] : Mx[9] : My[10] : Mz[11] : ...
%   alt(vision)[12] : alt(raw)[13] : Ax[14] : Ay[15] : Az[16] ...
%   Wx[17] : Wy[18] : Wz[19]

%% Setting biases
% Constants found during calibration rounded to 3 decimal places
% Tested on ar drone 174...

function yaw = special_mod_yaw(angle)
    if angle > 180
        yaw = angle - 360;
    elseif angle <= -180
        yaw = angle + 360;
    else
        yaw = angle;
    end
end
% pitch & roll predetermined.
% yaw must be found in calibration 1:600 pts
% yaw is erratic but luckily relatively arbitrary
att_bias = [ -0.079 , 0.016 , 0];

yaw_data = nav_data(1:600 , 4);
yaw_data_no_neg = yaw_data.*(yaw_data >= 0) + (360+yaw_data).*(yaw_data < 0);
yaw_initial = mean(yaw_data_no_neg);
yaw_initial = special_mod_yaw(yaw_initial);
att_bias(3) = yaw_initial;
% theta , phi , psi -> psi , theta , phi
temp = att_bias;
att_bias = [temp(3) , temp(1) , temp(2)];

% Vx Vy Vz
vel_bias = [13.514 , 0.625 ,0];

% Mx My Mz
% Dynamic, depends on initial calibration
% Assume first 600 points (3 sec) is sufficient considering it is very
% gaussian
magnetos = nav_data(1:600 , 9:11);
mag_means = mean(magnetos);

% Ax Ay Az
% After converting from LSB into G and removing gravity (Az=1), determined mean
acc_bias = [0.095 , -0.032 , 0.007];

% Wx Wy Wz
gyro_bias = [28.763 , 60.581 , -14.401];


%% Create unique variables
temp = nav_data(: , 2:4);
attitude = [temp(:,3) , temp(:,1) , temp(:,2)];
altitude = nav_data(:,5);
velocity = nav_data(: , 6:8);
magneto = nav_data(: , 9:10);
accel_no_grav = (nav_data(:, 14:16) - 2048) / 512;
accel_normalized = (nav_data( : , 14:16) - 2048) / 512;
ang_velocity = nav_data(:, 17:19);

%% Subtracting bias & Converting to nice coordinate system
% total_mat format:
% time[1] : pitch[2] : roll[3] : yaw[4] : alt(demo)[5] : ...
%   Vx[6] : Vy[7] :Vz[8] : Mx[9] : My[10] : Mz[11] : ...
%   alt(vision)[12] : alt(raw)[13] : Ax[14] : Ay[15] : Az[16] ...
%   Wx[17] : Wy[18] : Wz[19]


% psi , theta , phi
attitude = attitude - att_bias;
for i =1:length(attitude)
    attitude(i,1) = special_mod_yaw(attitude(i,1));
end
% subtract bias
velocity = velocity - vel_bias;

% normalize magneto by row
magneto = normr(magneto);
% determined initial heading
mag_initial = rad2deg(atan2(mag_means(2),mag_means(1)));

% Subtract bias and gravity
% make positive Az upwards
accel_no_grav = accel_no_grav - acc_bias;
accel_no_grav(:,3) = -accel_no_grav(:,3) + 1;

% for direction calcs
accel_normalized = accel_normalized - acc_bias;
accel_normalized = normr(accel_normalized);

% subtract bias
% adjust orientation to fit yaw/pitch/roll as given by API
ang_velocity = ang_velocity - gyro_bias;
ang_velocity(:,2) = -ang_velocity(:,2);
ang_velocity(:,3) = -ang_velocity(:,3)

%% Convert units
% cm -> m
altitude = altitude / 100;

%mm/s -> m/s
velocity = velocity / 10^3;

% G -> m/s^2
accel_no_grav = accel_no_grav*9.81;

%% make column vecs

attitude = attitude';
altitude = altitude';
velocity = velocity';
magneto = magneto';
accel_no_grav = accel_no_grav';
accel_normalized = accel_normalized';
ang_velocity = ang_velocity';

end