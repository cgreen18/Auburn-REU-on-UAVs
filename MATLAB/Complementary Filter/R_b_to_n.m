function Rmat = R_b_to_n(euler_vec)
    % euler_vec is in order: psi , theta , phi

    euler_vec = deg2rad(euler_vec);
    psi = euler_vec(1);
    theta = euler_vec(2);
    phi = euler_vec(3);

    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    
    Rmat = R_z*R_y*R_x;
    % R_mat(3,1) = -sin(theta)
    % -> theta = -arcsin(R_mat(3,1))
    
    % R_mat(3,2)/R_mat(3,3) = tan(psi)
    % -> psi = atan2(R_mat(3,2),R_mat(3,3))
    % or to avoid error, psi = atan2(R32/cos(theta) , R33/cos(theta))
    
    % R_mat(2,1) / R_mat(1,1) = tan(phi)
    % phi = atan2(r21/cos(theta) , R11/cos(theta))
end