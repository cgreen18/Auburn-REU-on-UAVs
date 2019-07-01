function vel_est = weight_velocity(weights , vel_sens, vel_dead_reck)

    vel_est = weights{1}*vel_sens + weights{2}*vel_dead_reck;

end