function [pt_cloud_data,N_Frames,pico_times] = grab_pico_data(filename)
    disp('Gathering Data')
    manager = royale.CameraManager();
    cameraDevice = manager.createCamera(filename);
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
    
    for ii = 1:N_Frames
        % retrieve data from camera
        try 
            pt_cloud_data{ii} = cameraDevice.getData();
            pico_times(ii,1) = pt_cloud_data{ii}.timeStamp/(10^6); 
        catch
            warning(strcat('Dataframe timeout, index :', num2str(ii)))
        end

    end 
    
    
end 