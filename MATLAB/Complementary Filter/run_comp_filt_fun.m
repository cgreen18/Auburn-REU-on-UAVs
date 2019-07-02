close all;
clear all;
clc;

filename = '7_1_clean10_5fps_reformatted.txt'

[time , d_pos , d_vel , d_att , s_pos , s_att] = comp_filt_fun(filename);

num_pts = length(d_pos);

close all;

figure;
plot(1:num_pts,time);

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


total = [d_pos , d_vel , d_att , s_pos , s_att];
total_table = array2table(total);
writetable(total_table,'7_1_clean10_5fps_succinct.txt' , 'WriteVariableNames',false);
