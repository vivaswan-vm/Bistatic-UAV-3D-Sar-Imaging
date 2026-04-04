function [Y_downlink_noise,nVar,snr_db] = Noise_addition(tx_channel,struc_par)

    % System Noise addition
    srx_volts = tx_channel(:);      
    % Signal Power
    sig_power = abs(srx_volts(1))^2;
    
    % Noise power (Watt)
    noise_power = struc_par.K*struc_par.Ts*struc_par.fs;
    % SNR (linear)
    snr_linear = sig_power/noise_power;
    % SNR (dB)
    snr_db = 10*log10(snr_linear);
    % Noise variance
    nVar = 1/snr_linear;
    srx_volts = repmat(srx_volts,1,1);
    % Generation of random noise 
    noise = sqrt(noise_power/2) *(randn( length(srx_volts),1) + 1j*randn( length(srx_volts),1));
    % Addition of system noise to the received signal at MU
    Y_downlink_noise = srx_volts + noise;

end