function angles = euler_from_R_n_to_b(R)
% Rotation matrix is 3x3
% R_mat(3,1) = -sin(theta)
% -> theta = -arcsin(R_mat(3,1))
    
% R_mat(3,2)/R_mat(3,3) = tan(psi)
% -> psi = atan2(R_mat(3,2),R_mat(3,3))
% or to avoid error, psi = atan2(R32/cos(theta) , R33/cos(theta))

% R_mat(2,1) / R_mat(1,1) = tan(phi)
% phi = atan2(r21/cos(theta) , R11/cos(theta))

% Assume abs(theta) < 90 degrees
% i.e. cos(theta) =/= 0
% If not, the quadcopter might be doing a backflip

R = R';

theta = -arcsin(R(3,1));

psi = atan2(R(3,2) / cos(theta) , R(3,3) / cos(theta));

phi = atan2(R(2,1) / cos(theta) , R(1,1) / cos(theta));

theta = rad2deg(theta);
psi = rad2deg(psi);
phi = rad2deg(phi);

% euler_vec is in order: psi , theta , phi
angles = [psi , theta, phi];
end