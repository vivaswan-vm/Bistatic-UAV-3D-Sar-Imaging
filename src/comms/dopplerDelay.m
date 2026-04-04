function tx_doppler = dopplerDelay(tx_beamformed,azimuth_MU,struc_constants,struc_packet_par)

% % Doppler Delay 
% Fast time axes
Lvec = (0:length(tx_beamformed)-1)*(struc_constants.ts);
% Slow time axes
Mvec = (0:struc_constants.PTM_len-1)*struc_packet_par.frame_dur;
Larray = repmat(Lvec',[1 struc_constants.PTM_len]);
Marray = repmat(Mvec,[length(tx_beamformed) 1]);
% Combined slow and fast time axes
time_array = Larray + Marray;
% Inside the exponential
Doppler_ele = 2*pi*struc_constants.uav_relative_velocity*cosd(azimuth_MU)*struc_constants.lambda_inv;
% Exponential
DopplerDelay = exp(1j*(Doppler_ele*time_array));
% Applying doppler effect on the packet   
tx_doppler = tx_beamformed .* DopplerDelay;
    
end