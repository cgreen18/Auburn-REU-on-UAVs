function vel_k = dead_reckoning_vel(delta_t , vel_km1 , acc_km1 , att_km1)
    % where vel and att in {n} frame and acc in {b} frame
    vel_k = vel_km1 + delta_t*R_b_to_n(att_km1)*acc_km1;
end