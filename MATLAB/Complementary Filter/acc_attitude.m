function attitude = acc_attitude(acc)
    pitch = rad2deg( atan2(acc(3),acc(1)) ) - 90;
    roll = rad2deg( atan2(acc(3),acc(2))) - 90;
    
    % psi , theta , phi
    attitude = [0 ; pitch ; roll];
end