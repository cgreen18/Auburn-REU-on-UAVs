function cloud_array = render_clouds(pt_cloud_data,N_Frames)
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
        cloud_array{ii} = pcdenoise(cloud_array{ii}, 'NumNeighbors', 4,'Threshold', 1); % de-noise, uncommenting this line will produce faster but messier clouds
    end 
end 