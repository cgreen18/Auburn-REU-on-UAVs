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

dummy_x = 1:num_pts;

%% Time
close all;
time_fig = figure('NumberTitle', 'off', 'Name', 'Time');
plot(dummy_x , total(:,1));

%% Attitude
% pitch[2] : roll[3] : yaw[4]
close all;
att_fig = figure('NumberTitle', 'off', 'Name', 'Attitude Estimation Line Graphs and Histograms');
set(att_fig,'Color','w');

%Pitch
subplot(2,3,1);
plot(dummy_x , total(:,2));
title("Pitch");
xlabel("Time [samples]");
ylabel("Estimated Pitch [deg.]");
xlim([0 , num_pts])

subplot(2,3,4);
histogram(total(:,2),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Pitch [deg.]");
ylabel("Relative Frequency");
xlim([-.2 , .1])

%Roll
subplot(2,3,2);
plot(dummy_x , total(:,3));
title("Roll");
xlabel("Time [samples]");
ylabel("Estimated Roll [deg.]");
xlim([0 , num_pts])

subplot(2,3,5);
histogram(total(:,3),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Roll [deg.]");
ylabel("Relative Frequency");
xlim([-.1 , .1])

%Yaw
subplot(2,3,3);
plot(dummy_x , total(:,4));
title("Yaw");
xlabel("Time [samples]");
ylabel("Estimated Yaw [deg.]");
xlim([0 , num_pts])
ylim([173.5 , 174.4]);

subplot(2,3,6);
histogram(total(:,4),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Yaw [deg.]");
ylabel("Relative Frequency");
xlim([ 173.6 , 174.3])



%% Velocities
% Vx[6] : Vy[7] :Vz[8]
close all;
vel_fig = figure('NumberTitle', 'off', 'Name', 'Velocity Estimation in XYZ Line Graphs and Histograms');
set(vel_fig,'Color','w');

%Vx
subplot(2,3,1);
plot(dummy_x , total(:,6));
title("V_x");
xlabel("Data Points");
ylabel("Estimated Velocity (mm/s)");
xlim([0 , num_pts])

subplot(2,3,4);
histogram(total(:,6),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Velocity (mm/s)");
ylabel("Frequency");
xlim([5, 20])

%Vy
subplot(2,3,2);
plot(dummy_x , total(:,7));
title("V_y");
xlabel("Data Points");
ylabel("Estimated Velocity (mm/s)");
xlim([0 , num_pts])

subplot(2,3,5);
histogram(total(:,7),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Velocity (mm/s)");
ylabel("Frequency");
xlim([-10, 10])

%Vz
subplot(2,3,3);
plot(dummy_x , total(:,8));
title("V_z (Broken)");
xlabel("Data Points");
ylabel("Estimated Velocity (mm/s)");
xlim([0 , num_pts])
ylim([-.6 , .6])

subplot(2,3,6);
histogram(total(:,8),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Velocity (mm/s)");
ylabel("Frequency");
xlim([-.1,.1])


%% Magnetometer Readings
% Mx[9] : My[10] : Mz[11]
close all;

mag_fig = figure('NumberTitle', 'off', 'Name', 'Magnetometer Sensor Line Graphs and Histograms');
set(mag_fig,'Color','w');

%Mx
subplot(2,3,1);
plot(dummy_x , total(:,9));
title("M_x");
xlabel("Data Points");
ylabel("Magnetic Induction [mT]");
xlim([0 , num_pts])

subplot(2,3,4);
histogram(total(:,9),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Magnetic Induction [mT]");
ylabel("Frequency");


%My
subplot(2,3,2);
plot(dummy_x , total(:,10));
title("M_y");
xlabel("Data Points");
ylabel("Magnetic Induction [mT]");
xlim([0 , num_pts])

subplot(2,3,5);
histogram(total(:,10),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Magnetic Induction [mT]");
ylabel("Frequency");


%Mz
subplot(2,3,3);
plot(dummy_x , total(:,11));
title("M_z");
xlabel("Data Points");
ylabel("Magnetic Induction [mT]");
xlim([0 , num_pts])

subplot(2,3,6);
histogram(total(:,11),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Magnetic Induction [mT]");
ylabel("Frequency");

%% Altitude Readings

%%%%%%
% Unfinished
% 
%%%%%%

% alt(demo)[5] : alt(vision)[12] : alt(raw)[13]
close all;

alt_fig = figure('NumberTitle', 'off', 'Name', 'Megnetometer Sensor Line Graphs and Histograms');
set(alt_fig,'Color','w');

%Pitch
subplot(2,3,1);
plot(dummy_x , total(:,2));
title("Pitch");
xlabel("Time [samples]");
ylabel("Estimated Pitch [deg.]");
xlim([0 , num_pts])

subplot(2,3,4);
histogram(total(:,11),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Pitch (deg.)");
ylabel("Relative Frequency");
xlim([-.2 , .1])

%Roll
subplot(2,3,2);
plot(dummy_x , total(:,3));
title("Roll");
xlabel("Data Points");
ylabel("Estimated Roll (deg.)");
xlim([0 , num_pts])

subplot(2,3,5);
histogram(total(:,11),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Roll (deg.)");
ylabel("Relative Frequency");
xlim([-.1 , .1])

%Yaw
subplot(2,3,3);
plot(dummy_x , total(:,4));
title("Yaw");
xlabel("Data Points");
ylabel("Estimated Yaw (deg.)");
xlim([0 , num_pts])
ylim([173.5 , 174.4]);


subplot(2,3,6);
histogram(total(:,11),100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Estimated Yaw (deg.)");
ylabel("Relative Frequency");

%% Accelerometer Readings
% Ax[14] : Ay[15] : Az[16]
close all;

acc_fig = figure('NumberTitle', 'off', 'Name', 'Accelerometer Sensor Line Graphs and Histograms');
set(acc_fig,'Color','w');

%Ax
subplot(2,3,1);
Ax = (total(:,14) - 2048)*(1000/512);
plot(dummy_x , Ax);
title("a_x");
xlabel("Time [samples]");
ylabel("Acceleration [mG]");
xlim([0 , num_pts])

subplot(2,3,4);
histogram(Ax,100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Acceleration [mG]");
ylabel("Relative Frequency");
xlim([70 , 110]);

%Ay
subplot(2,3,2);
Ay = (total(:,15) - 2048)*(1000/512);
plot(dummy_x , Ay);
title("a_y");
xlabel("Time [samples]");
ylabel("Acceleration [mG]");
xlim([0 , num_pts])

subplot(2,3,5);
histogram(Ay,100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Acceleration [mG]");
ylabel("Relative Frequency");
xlim([-50 , -10]);

%Az
subplot(2,3,3);
Az = (total(:,16) - 2048)*(1000/512);
plot(dummy_x ,Az);
title("a_z");
xlabel("Time [samples]");
ylabel("Acceleration [mG]");
xlim([0 , num_pts])


subplot(2,3,6);
histogram(Az,100000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Acceleration [mG]");
ylabel("Relative Frequency");
xlim([980 , 1020]);


%% Gyroscope Readings
% Wx[17] : Wy[18] : Wz[19]
close all;

acc_fig = figure('NumberTitle', 'off', 'Name', 'Gyroscope Sensor Line Graphs and Histograms');
set(acc_fig,'Color','w');

%Wx
subplot(2,3,1);
plot(dummy_x , total(:,17));
title("\omega_x (Roll)");
xlabel("Time [samples]");
ylabel("Angular Velocity [deg/s]");
xlim([0 , num_pts])

subplot(2,3,4);
histogram(total(:,17),1000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Angular Velocity [deg/s]");
ylabel("Relative Frequency");
xlim([18,42]);

%Wy
subplot(2,3,2);
plot(dummy_x , total(:,18));
title("\omega_y (Pitch)");
xlabel("Time [samples]");
ylabel("Angular Velocity [deg/s]");
xlim([0 , num_pts])

subplot(2,3,5);
histogram(total(:,18),1000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Angular Velocity [deg/s]");
ylabel("Relative Frequency");
xlim([44 , 78])

%Wz
subplot(2,3,3);
plot(dummy_x ,total(:,19));
title("\omega_z (Yaw)");
xlabel("Time [samples]");
ylabel("Angular Velocity [deg/s]");
xlim([0 , num_pts])


subplot(2,3,6);
histogram(total(:,19),1000,'Normalization','probability','FaceColor',[0 0.4470 0.7410],'EdgeColor',[0 0.4470 0.7410]);
xlabel("Angular Velocity [deg/s]");
ylabel("Relative Frequency");
xlim([-22 , -8])

