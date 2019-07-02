function vel_sens = sensor_velocity(vel_raw , att)
    vel_sens = R_b_to_n(att)*vel_raw;
end