clear all;
clc;
close all;


filename = '7_10_2calib_hallway_roll2_data.txt'

calib_time = 2;
threshold_factor = 3;
cutoff_freq = .01;

[time , drone_pos , drone_vel , drone_att , lidar_pos , lidar_att] = temp_navdata_filtering(filename , calib_time, threshold_factor , cutoff_freq);

plot3D_drone(drone_pos);

plot2D_drone_multiplot(drone_pos , drone_vel , drone_att);


function plot3D_drone(d_pos)

figure;
scatter3(d_pos(:,1),d_pos(:,2),d_pos(:,3),'.');
xlabel('x');
ylabel('y');
axis([-10,50,-30,30,0,4]);
axis vis3d;

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