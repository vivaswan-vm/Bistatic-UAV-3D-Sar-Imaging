function rxPSDU = dataProcessing(Y_downlink,dmg,nVar,struc_ofdm_info,struc_packet_par)
    
    % OFDM information
    ofdmInfo = wlanDMGOFDMInfo();   
    % At MU
    % Extraction of Data bits
    rx_packet = Y_downlink;
   
    % Extract data field
    rxData = rx_packet(struc_packet_par.fieldIndices.DMGData(1):struc_packet_par.fieldIndices.DMGData(2),:);

    % OFDM demodulate
    inputIn3D = reshape(rxData,(struc_ofdm_info.fftLen+struc_ofdm_info.cpLen(1)),struc_ofdm_info.numSym,1);
  
    % After CP removal
    postCPRemoval = inputIn3D([struc_ofdm_info.cpLen(1)+1:struc_ofdm_info.fftLen+struc_ofdm_info.symOffset(1),struc_ofdm_info.symOffset(1)+1:struc_ofdm_info.cpLen(1)],:,:);

    % FFT
    postFFT = fft(postCPRemoval,[],1);

    % FFT shift
    fftout = fftshift(postFFT,1);

    % Extract active subcarriers from full FFT
    sym = fftout(ofdmInfo.ActiveFFTIndices,:,:);

    % Scale the demodulated waveform
    demodSym = sym*sqrt(ofdmInfo.NumTones)/ofdmInfo.FFTLength;

    % Extract pilot symbols for channel estimation (16 pilots)
    pilot_symbols_received = demodSym(ofdmInfo.PilotIndices,:,:);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Least Square Channel Estimation
    % The transmitted pilots are stored 
    pilot_symbols_transmitted = get_transmitted_pilots(struc_packet_par.NSYM);
    % The channel is estimated using the pilots
    H_estimate = pilot_symbols_received./pilot_symbols_transmitted; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Interpolation
    % pilot positions
    pilot_ind = ofdmInfo.PilotIndices;  
    % Discard pilots
    dataSym = demodSym(ofdmInfo.DataIndices,:,:);     
    % H estimate after interpolation 
    H_LS = interpolate_optimized(H_estimate,pilot_ind',struc_ofdm_info.fftLen,'linear');
    % Extracting the estimates only for the data subcarriers
    H_estimate_data = H_LS(ofdmInfo.DataIndices,:,:);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Equalization
    % The channel estimates for the data sub carriers
    chanSym = H_estimate_data;
    % Repeating the noise var to create a 3D array where the 3rd dimension
    % is number of iterations for averaging
    B = repmat(nVar.',1,size(dataSym,2),size(dataSym,1)); % use repmat to create a 3x10x10 copy
    nVar_rep = permute(B,[3 2 1]); % permute to the correct order
    % MMSE Equalization of the received data symbols
    [eqSym,~] = MMSE_Equalize_optimized(dataSym,chanSym,nVar_rep);
   
    % Recovering the transmitted bits   
    % Recover data (qpsk demod + ldpc decoding + descrambling)
    rxPSDU = wlanDMGDataBitRecover(eqSym(:,:,1),nVar,dmg);  
    
end