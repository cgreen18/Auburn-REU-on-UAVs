function attitude = mag_attitude(mag , mag_initial)
    yaw = rad2deg( atan2(mag(2), mag(1) )) - mag_initial;
    
    % psi , theta , phi
    attitude = [yaw ; 0 ; 0];
end