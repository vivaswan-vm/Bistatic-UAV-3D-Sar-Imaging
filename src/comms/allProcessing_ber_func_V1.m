function [ber,snr_db,rxPSDU] = allProcessing_ber_func_V1(tx,txPSDU,trajectory_angle,range_tgt,...
   struc_constants,struc_packet_par,struc_ofdm_info,dmg)

% Angle between MU and BS (degrees)
azimuth_MU = 90 - trajectory_angle;

%Packet after beamforming at BS and MU
tx_beamformed = tx;%beamforming_jrcV1(tx,BS_beamform_angle,MU_beamform_angle,trajectory_angle,struc_constants);

% Doppler delay addition
tx_doppler = dopplerDelay(tx_beamformed,azimuth_MU,struc_constants,struc_packet_par);

% Packet passes through channel
tx_channel = channelmodelling(tx_doppler,range_tgt,struc_constants);

% System noise addition
[tx_noise,nVar,snr_db] = Noise_addition(tx_channel,struc_constants);

% Data processing to get received bits
rxPSDU = dataProcessing(tx_noise,dmg,nVar,struc_ofdm_info,struc_packet_par);

% Calculation of BER
[~,ber] = biterr(txPSDU,rxPSDU);%ber = nerrors/total no. of bits

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end