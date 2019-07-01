function att_k = dead_reckoning_att(delta_t , att_km1 , ang_vel_km1)
    % attitude in {n} and angular velocity in {b}
    
    
%     R_z = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
%     R_y = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
%     R_x = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    
    %convenience
    theta = att_km1(2);
    phi = att_km1(3);
    psi = att_km1(1);
    omega = ang_vel_km1;
    
    c = @(angle) (cos(deg2rad(angle)));
    s = @(angle) (sin(deg2rad(angle)));
    t = @(angle) (tan(deg2rad(angle)));
    
    dtheta = omega(2)*c(theta)-omega(3)*s(phi);
    dphi = omega(1) + omega(2)*s(phi)*t(theta) + omega(3)*c(phi)*t(theta);
    dpsi = omega(2)*(s(phi)/c(theta)) + omega(3)*(c(phi)/c(theta));
    
    % psi , theta , phi
    datt = [dpsi ; dtheta ; dphi];
    
    att_k = att_km1 + delta_t*datt;
end