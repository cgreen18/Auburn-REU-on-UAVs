function ptCloudScene = process_scene(pico_FileName, time, sensor_pos,sensor_att,nav_data_trust,start_here,end_here,use_rotations,use_translations,refresh_data)
% Lets stitch baby, this first block gathers and syncs data
tic
    sensor_att = movmean(sensor_att, 41,1);
    % Grab necessary data from pico flexx
    if refresh_data
        [pt_cloud_data,N_Frames,pico_times] = grab_pico_data(pico_FileName);
        save('temp_stitch_variables.mat','pt_cloud_data','N_Frames','pico_times') %save these to workspace 
    else 
        load('temp_stitch_variables.mat','pt_cloud_data','N_Frames','pico_times')
    end 
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
    
    % Grab navigation data we will be using 
    filtered_nav_pos_data = zeros(size(sensor_pos,1),7);
    filtered_nav_pos_data(:,1) = time; % time
    filtered_nav_pos_data(:,2) = sensor_att(:,2); % pitch
    filtered_nav_pos_data(:,3) = sensor_att(:,3); % roll
    filtered_nav_pos_data(:,4) = sensor_att(:,1); % yaw
    filtered_nav_pos_data(:,5) = sensor_pos(:,1); % x
    filtered_nav_pos_data(:,6) = sensor_pos(:,2); % y
    filtered_nav_pos_data(:,7) = sensor_pos(:,3); % z

    %%%%%%%%%%%%%%%% Sync Navigation Data with pico 
    disp('syncing data')
    
    threshold = .05;
    previous_threshold = threshold; %seconds 
    differenceFrames = zeros(N_Frames, size(filtered_nav_pos_data,2)); 
    %init total navigation date columns are syncronized time, yaw, pitch, roll,
    %etc
    synced_navigation_data = zeros(N_Frames, size(filtered_nav_pos_data,2)); 

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
    running_average = ((double(ii)-1)/double(ii))*running_average + ((1/double(ii))*differenceFrames(ii,4));
    if abs(differenceFrames(ii+1,4)) >= abs(running_average*3)
       differenceFrames(ii+1,4) = running_average; 
    end
end 
%render the clouds in this function
if refresh_data
    cloud_array = render_clouds(pt_cloud_data,N_Frames);
    save('temp_cloud_data.mat','cloud_array')
else 
    load('temp_cloud_data.mat','cloud_array')
end 
    
disp('transforming and stitching clouds')
mergeSize = 0.001;
gridSize = 0.001;
    
if end_here > N_Frames
    end_here=N_Frames;
end 

start_frame = start_here; end_frame = end_here;
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
    
    for ii = start_frame:end_frame
        ptCloudRef = ptCloudScene;
        ptCloudCurrent = cloud_array{ii};
        %%%%%%%%%%%%%%%%%%%% Get Transforms here
        %rotations
        rotation_matrix = eye(4);
        if use_rotations
            euler_angle=differenceFrames(ii,2:4).*nav_trust_weight;
            rotation_matrix = euler2rot(euler_angle);
        end 
        %translations
        translation_matrix = eye(4);
        if use_translations
            translation_matrix(1:3,4) = differenceFrames(ii,5:7)';
        end
        % form the total custom transform from the navigation data 
        custom_tform = affine3d(inv(rotation_matrix)*inv(translation_matrix'));

        tform = affine3d(eye(4));
        if ~(nav_trust_weight == 1)
        %%%%%%%%%%% ICP Portion
            fixed = pcdownsample(cloud_array{ii-1}, 'gridAverage', gridSize);
            moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
            tform = pcregistericp(moving, fixed,'Extrapolate',true, 'Metric','pointToPlane','Tolerance',[0.01, 0.05]);
        end 
        %%%%%%%%%% Merge all transforms
        accum_custom_tform = affine3d((custom_tform.T + tform.T.*icp_weight_matrix - eye(4)) * accum_custom_tform.T); %transformation accumulator
%         isRigid(accum_custom_tform)
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
toc
end 