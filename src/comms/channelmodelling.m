function [tx_channel] = channelmodelling(tx_doppler,range_tgt,struc_constants)

% if(strcmp(struc_constants.channelType,'friis'))
    % Friis free space
    
    % Friis coefficient
    friis_coeff=(sqrt(struc_constants.Ptx*32*32)*struc_constants.lambda)/(4*pi*range_tgt);

    % Received signal without noise
    tx_channel = friis_coeff*tx_doppler;    
% else
%     % Rician channel
%     k = 0.1;
%     % Generate random NLOS components
%     Hw = struc_constants.mu+sqrt(struc_constants.variance)*randn(size(tx_doppler)); 
% 
% 
% %     % Channel Modelling (Rician channel) 
% %     H = sqrt(struc_constants.ricianFactor/(struc_constants.ricianFactor+1)).* tx_doppler...
% %         + sqrt(1/(struc_constants.ricianFactor+1)).*Hw;
% %     % Friis coefficient
% %     friis_coeff=(sqrt(struc_constants.Ptx)*struc_constants.lambda)/(4*pi*range_tgt);
% %     % Received signal without noise
% %     tx_channel =  friis_coeff*H;
%    friis_coeff=(sqrt(struc_constants.Ptx)*struc_constants.lambda)/(4*pi*range_tgt);
% % Channel Modelling (Rician channel) 
%     H_los = (sqrt(struc_constants.ricianFactor/(struc_constants.ricianFactor+1)).* tx_doppler)*friis_coeff;
%     H_nlos =  ((sqrt(1/(struc_constants.ricianFactor+1)).*Hw)*friis_coeff+(tx_doppler*friis_coeff*k));
%     % Friis coefficient
% 
%     % Received signal without noise
%     tx_channel =  H_los+H_nlos;  
end