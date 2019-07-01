function pos_k = dead_reckoning_pos(delta_t , pos_km1 , vel_km1 , acc_km1 , att_km1)
    % pos , att, and vel in {n} frame of reference and acc in {b} frame
    pos_k = pos_km1 + delta_t*vel_km1 + .5*(delta_t^2)*R_b_to_n(att_km1)*acc_km1;
end