function [ofdm_info,packet_par] = initialisation_frame_parameters_func(PSDULength,dmg,struc_constants)

    % Number of data bits in each packet
    packet_par.databits = PSDULength * 8;
    % Indices of fields within the packet
    packet_par.fieldIndices = wlanFieldIndices(dmg);
    % Code rate for demodulation and decoding
    packet_par.rate = 3/4;
    % Length of the code word
    packet_par.LCW = 672;
    % Number of coded bits per symbol
    packet_par.NCBPS = 672;
    
    % Different number of symbols based on whether the trn fields are present
    % or not
%   Calculating the number of symbols in each packet based on the
        % number of bytes transmitted
        packet_par.NSYM = ceil((PSDULength * 8)/(packet_par.LCW * packet_par.rate));
        % Calculating the number of samples in each packet based on the
        % number of symbols
        packet_par.num_samples_packet = double(packet_par.fieldIndices.DMGHeader(2) + 640*packet_par.NSYM);
        % Frame duration
        packet_par.frame_dur = struc_constants.ts * packet_par.num_samples_packet;
   
    
    % Initializing ofdm information
    % Number of data + pilot subcarriers
    ofdm_info.NTones = 352;
    % Number of data carrying subcarriers
    ofdm_info.Nsd = 336;
    % FFT length
    ofdm_info.fftLen = 512;
    % Cyclic prefix length
    ofdm_info.cpLen = 128;
    % Number of symbols in each packet
    ofdm_info.numSym =  packet_par.NSYM;
    ofdm_info.numRx = 1;
    ofdm_info.numTx = 1;
    % Symbol offset
    ofdm_info.symOffset = 126;
    
end