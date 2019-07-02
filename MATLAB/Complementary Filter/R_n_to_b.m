function Rmat = R_n_to_b(euler_vec)
    %or transpose of R_b_to_n but nice to have both explicitly written

    euler_vec = deg2rad(euler_vec);
    psi = euler_vec(1);
    theta = euler_vec(2);
    phi = euler_vec(3);

    R_z = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    R_y = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    R_x = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    
    Rmat = R_z*R_y*R_x;
    % R_mat(1,3) = -sin(theta)
end