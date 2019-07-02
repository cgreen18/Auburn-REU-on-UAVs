%% Complementary Filter. Goal: All sensor data -> p , v , theta (attitude)
% Complementary Filter function - comp_filt_fun(name of file) -> drone...
% position, velocity, and attitude & sensor position and attitude

function [d_pos , d_vel , d_att , s_pos , s_att] = comp_filt_fun(filename)

    %% Read in file
    nav_data = read_navdata(filename);
    [ num_pts , num_sens ] = size(nav_data);
    delta_t = nav_data(2,1)-nav_data(1,1);
    
    [attitude_sensor , yaw_initial , altitude , velocity_sensor , magneto , mag_init_angle , accel_no_grav , accel_normalized , ang_velocity] = clean_data(nav_data);

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

    % Trust in: theta_sens , dead_reckoning , accel/mag
    weights_att = { .6*eye(3) ; .3*eye(3) ; .1*eye(3)};
    % trust in: sensor , dead_reckoning
    weights_vel = {[.3 ,0 , 0; 0 , .7,0;0,0,0] ; [.7,0,0;0,.3,0;0,0,1]};
    % trust in: alt sens , dead reckoning
    weights_pos = {[0,0,0;0,0,0;0,0,.9] ; [1,0,0;0,1,0;0,0,.1] };

    for k = 2:num_to_filter

        if mod(k,100) == 0
            fprintf('Processing drone point: %d\n',k)
        end

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
        pos_dead = dead_reckoning_pos(delta_t , position(: , k-1) , velocity(: , k-1) , accel_no_grav(: , k-1) , attitude(: , k-1) );

        % weights , alt , pos_dead_wreck
        pos_est = weight_position(weights_pos , [0;0;altitude( k-1 )] , pos_dead );
        position(:,k) = pos_est;

        %% Processing Velocity
        % delta_t , vel_km1 , acc_km1 , att_km1
        vel_dead = dead_reckoning_vel(delta_t , velocity(: , k-1) , accel_no_grav(:,k-1) , attitude(: , k-1));
        % weights , vel_sens, vel_dead_reck
        vel_sens = sensor_velocity(velocity_sensor(: , k) , attitude(: , k) );
        
        vel_est = weight_velocity(weights_vel , vel_sens , vel_dead );
        velocity(:, k) = vel_est;

    end   
    
    
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
    d_pos = position';
    d_vel = velocity';
    d_att = attitude';
    
    s_pos = sensor_pos';
    s_att = sensor_att';
    
    
end