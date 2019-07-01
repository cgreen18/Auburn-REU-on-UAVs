function navdata = read_navdata()
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

    %[n , m] = size(file_1_data);
    %file_1_data = file_1_data(10000:n , :);

    %[n , m] = size(file_2_data);
    %file_2_data = file_2_data(10000:n , :);

    navdata = [ file_1_data ; file_2_data ];
end