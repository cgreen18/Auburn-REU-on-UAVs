function att_est = weight_attitude(weights, att_dead , att_acc , att_mag , att_sens)
    % combine non-interfering elements
    att_mag_acc = att_mag + att_acc;
    
    att_est = weights{1}*att_sens + weights{2}*att_dead + weights{3}*att_mag_acc;
end