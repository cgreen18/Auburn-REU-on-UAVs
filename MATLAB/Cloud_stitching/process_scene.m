function ptCloudScene = process_scene(pico_FileName, time, sensor_pos,sensor_att,nav_data_trust)
% Lets stitch baby, this first block gathers and syncs data
    sensor_att = movmean(sensor_att, 41,1);
    disp('Init camera')
    %%%%%%%%% First Initialize arrays
    % open recorded camera file
    manager = royale.CameraManager();
    cameraDevice = manager.createCamera(pico_FileName);
    delete(manager);
    cameraDevice.initialize();
    % configure playback
    % cameraDevice.loop(false);
     cameraDevice.useTimestamps(true);
    N_Frames=cameraDevice.frameCount();
    % start capture mode
    cameraDevice.startCapture();
    pt_cloud_data = cell(1,N_Frames); 
    pico_times = zeros(N_Frames,2); 


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gather and syncronize data
    disp('Gathering data')
    for ii = 1:N_Frames
        % retrieve data from camera
        pt_cloud_data{ii} = cameraDevice.getData();
        % retrieve times
        pico_times(ii,1) = pt_cloud_data{ii}.timeStamp/(10^6); 
    end 
%     plot(1:N_Frames, pico_times(:,1),'.','MarkerSize',9)
% % % %     pico_times(:,1) = pico_times(:,1) - pico_times(1,1);
    % fix pico times here
    time_multiplier = 1; 
    fixed_times = zeros(N_Frames,1);
    for ii = 2:N_Frames
        if pico_times(ii,1) == pico_times(ii-1,1)
            fixed_times(ii) = pico_times(ii,1) + 0.2*time_multiplier; 
            time_multiplier = time_multiplier + 1; 
        else
            fixed_times(ii) = pico_times(ii,1);
            time_multiplier = 1; 
        end
    end
    pico_times(:,1) = fixed_times; 


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIX YEEHAW
% %     yaw = sensor_att(:,1);
% %     for tt = 1:length(yaw)
% %         if yaw(tt) < 0
% %             yaw(tt) = 360+yaw(tt);     
% %         end
% %         
% %     end

    filtered_nav_pos_data = zeros(size(sensor_pos,1),7);
    filtered_nav_pos_data(:,1) = time; % time
    filtered_nav_pos_data(:,2) = sensor_att(:,2); % pitch
    filtered_nav_pos_data(:,3) = sensor_att(:,3); % roll
    filtered_nav_pos_data(:,4) = sensor_att(:,1); % yaw
    filtered_nav_pos_data(:,5) = sensor_pos(:,1); % x
    filtered_nav_pos_data(:,6) = sensor_pos(:,2); % y
    filtered_nav_pos_data(:,7) = sensor_pos(:,3); % z

    %%%%%%%%%%%%%%%% Sync Navigation Data
    disp('syncing data')
    
    threshold = .05;
    previous_threshold = threshold; %seconds 
    differenceFrames = zeros(N_Frames, size(filtered_nav_pos_data,2)); 
    %init total navigation date columns are syncronized time, yaw, pitch, roll,
    %etc
    synced_navigation_data = zeros(N_Frames, size(filtered_nav_pos_data,2)); 
% % % %     nav_times = filtered_nav_pos_data(:,1) - filtered_nav_pos_data(1,1);
%     pico_times(1:1)
% Sync Method 1
    for ii = 2:N_Frames 
        current_time = pico_times(ii,1); 
        for jj = 1:length(time)
            if abs(current_time - time(jj)) < previous_threshold
                %save the index of the syncronized nav_data
                pico_times(ii,2) = jj;
                synced_navigation_data(ii,1) = current_time;
                synced_navigation_data(ii,2:7) = filtered_nav_pos_data(jj,2:7);
                differenceFrames(ii,:) = synced_navigation_data(ii,:) - synced_navigation_data(ii-1,:);
                previous_threshold = abs(current_time - time(jj)); 
            end 

        end

        previous_threshold = threshold;
    end 
% % % % %     Sync Method 2
% % % % previous_threshold = 0.05; 
% % % % threshold = previous_threshold; 
% % % % for ii = 1:length(time)
% % % %     current_time = pico_times(2,1);
% % % %     if abs(current_time - time(ii)) <= previous_threshold
% % % %         start_index = ii; 
% % % %         previous_threshold = abs(current_time - time(ii)); 
% % % %         disp(previous_threshold)
% % % %     end 
% % % % end 
% % % % 
% % % % jj=start_index; 
% % % % nav_incr = 40; 
% % % % for ii = 2:N_Frames
% % % %     pico_times(ii,2) = jj;
% % % %    synced_navigation_data(ii,1) = time(jj);
% % % %    synced_navigation_data(ii,2:7) = filtered_nav_pos_data(jj,2:7);
% % % %    differenceFrames(ii,:) = synced_navigation_data(ii,:) - synced_navigation_data(ii-1,:);
% % % %    jj = jj + nav_incr; 
% % % % end
% % % % disp('j')

%%% outlier prevention
running_average = sum(differenceFrames(10:15,4))/6; 
for ii = 16:N_Frames-1
    running_average = ((ii-1)/ii)*running_average + (1/ii)*differenceFrames(ii,4);
    if abs(differenceFrames(ii+1,4)) >= running_average*3
       differenceFrames(ii+1,4) = running_average; 
    end
end 

    % Render Point Clouds In this Block
    % create a cell array called cloud_array that will hold the point clouds 
    % instantiate
    cloud_array = cell(1,N_Frames); 
    disp('rendering clouds')
    for ii = 1:N_Frames
        XI = reshape(pt_cloud_data{ii}.x, [1,38304]);
        YI = reshape(pt_cloud_data{ii}.y, [1,38304]);
        ZI = reshape(pt_cloud_data{ii}.z, [1,38304]);
        temp_cloud = [XI; YI; ZI]';
        cloud_array{ii} = pointCloud(temp_cloud);
        cloud_array{ii}.Intensity = single(reshape(pt_cloud_data{ii}.grayValue,[1,38304]))';
        cloud_array{ii} = pcdenoise(cloud_array{ii}, 'NumNeighbors', 3,'Threshold', 1); % de-noise, uncommenting this line will produce faster but messier clouds
    end 

    disp('transforming and stitching clouds')
    mergeSize = 0.001;
    gridSize = 0.001;

    start_frame = 15; end_frame = N_Frames;
    nav_transformed_length = end_frame - start_frame; 
    nav_transformed_frames=cell(1,(nav_transformed_length));
    ptCloudScene = cloud_array{start_frame-1};

    %%%%%%%% Experimental weights for transforms
    nav_trust_weight = nav_data_trust; 
    icp_weight = 1-nav_trust_weight; 
    icp_weight_matrix = [1 icp_weight icp_weight icp_weight
                         icp_weight 1 icp_weight icp_weight
                         icp_weight icp_weight 1 icp_weight
                         icp_weight icp_weight icp_weight 1];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Loop will apply all transforms 
    accum_custom_tform = affine3d(eye(4));
    jj=1;
    %Start stitching
    for ii = start_frame:end_frame
        ptCloudRef = ptCloudScene;
        ptCloudCurrent = cloud_array{ii};
        %%%%%%%%%%%%%%%%%%%% Get Transform here, %currently pitch roll yaw only
        euler_angle=differenceFrames(ii,2:4).*nav_trust_weight;
    % %     euler_angle(1:3) = [1 0 1];
    % %     euler_angle

        custom_tform = affine3d(inv(euler2rot(euler_angle)));
        %%%%%%%%%%% ICP Portion
        fixed = pcdownsample(cloud_array{ii-1}, 'gridAverage', gridSize);
        moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
        tform = pcregistericp(moving, fixed,'Extrapolate',true, 'Metric','pointToPlane','Tolerance',[0.01, 0.05]);
        %%%%%%%%%% Merge all transforms
        accum_custom_tform = affine3d((custom_tform.T + tform.T.*icp_weight_matrix - eye(4)) * accum_custom_tform.T); %transformation accumulator
        %then perform the transformation we want 
        ptCloudAligned = pctransform(ptCloudCurrent,accum_custom_tform);
        nav_transformed_frames{jj} = ptCloudAligned;
        %%%%%%%%%%%%%%%%%%%% Merge here 

        ptCloudScene = pcmerge(ptCloudRef,ptCloudAligned,mergeSize);
        jj = jj+1;
    end


    % Euler to rotation matri, specialized for ARDrone to Pico Flexx
    % coordinate system conversion
    function rot_matrix = euler2rot(angles)
        phi = angles(1)*pi/180; theta = angles(2)*pi/180; psi = angles(3)*pi/180; 
        Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1]; 
        Ry = [cos(psi) 0 sin(psi); 0 1 0; -sin(psi) 0 cos(psi)];
        Rx = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
        rot_matrix = Rz*Ry*Rx; 
        rot_matrix = horzcat(vertcat(rot_matrix,[0 0 0]), [0 0 0 1]');
    end 

    disp('stitching finished')
end 