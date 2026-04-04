function [tx,dmg] = packet_generation(data,PSDULength)




    %Appending zeros to make data for N OFDM Symbols
    %No of complete OFDM symbols 
    %504 is the size of bits for 1 OFDM Symbol (initially) with 3/4 QPSK
    %Encoding
    dmg = wlanDMGConfig;
    dmg.MCS = 17;             % OFDM
    dmg.TrainingLength = 0; % Use 32 training subfields 
    dmg.PacketType = 'TRN-T'; % Transmitter training
    dmg.PSDULength = PSDULength;     % Bytes
     
    n = data.data_size_bits/504;

    if mod(data.data_size_bits,504)==0
        %Final data 
        n1 = floor(n);
        data.data = data.data_bits;
    else
        n1 = floor(n);
        %No.of zeros to be appended
        zeros_n = ((n1+1)*504)-data.data_size_bits;

        %Final data 
        data.data = [data.data_bits;zeros(zeros_n,1)];
    end

    %Total Number of OFDM Symbols
    N = length(data.data)/504;

    %Final Data
    %Calling the function for getting final waveform
    tx = windowing(data.data, data.data_size_bytes);
end
