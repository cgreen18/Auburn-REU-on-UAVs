function pos_est = weight_position(weights , alt , pos_dead_wreck)
    % weights is cell array of 3x3 matricies {w1 , w2}
    pos_est = weights{1}*alt + weights{2}*pos_dead_wreck;
end