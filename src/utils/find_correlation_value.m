function [corr_values_each_beam] = find_correlation_value(tx_short,BS_beamform_angle,MU_beamform_angle,...
    trajectory_angle,range_tgt,flagStages,struc_constants,struc_packet_par) 

% Sending omni packets and receiving at different angles at MU
% Angle between MU and BS (degrees)
azimuth_MU = 90 - trajectory_angle;

%Packet after beamforming at BS and MU
tx_beamformed = beamforming_jrcV2(tx_short,BS_beamform_angle,MU_beamform_angle,trajectory_angle,flagStages,struc_constants);

% Doppler delay addition
tx_doppler = dopplerDelay(tx_beamformed,azimuth_MU,struc_constants,struc_packet_par);

% Packet passes through channel
tx_channel = channelmodelling(tx_doppler,range_tgt,struc_constants);

% System noise addition
[tx_noise,~,~] = Noise_addition(tx_channel,struc_constants);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% correlation of CEFs to find the best receive beam for the MU
% Extract the CEF from the received packet at MU
tx_noise_cef = tx_noise(struc_packet_par.fieldIndices.DMGCE(1):struc_packet_par.fieldIndices.DMGCE(2));
tx_cef = tx_short(struc_packet_par.fieldIndices.DMGCE(1):struc_packet_par.fieldIndices.DMGCE(2));
% Correlation between the received CEF and the CEF without weights
corr = xcorr(tx_cef,tx_noise_cef);
% Finding the absolute value of the correlation value
corr_abs = abs(corr);
% Picking one correlation value
corr_values_each_beam = corr_abs(1);
        
end