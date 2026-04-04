%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Preamble Block for 802.11ad
%13/8/21
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clc
% close all;
% clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [P_6] = Preamble_part()
%Golay Sequence of length 128
[Ga, Gb] = wlanGolaySequence(128);

%STF

STF_1 = (repmat(Ga, 16,1));

STF = [STF_1; -Ga];
%%Golay Sequence of length 512
CEF = [-Gb; Ga; -Gb; -Ga; -Gb; -Ga; Gb; -Ga; -Gb];

%Preamble
P_1 = [STF;CEF];
% % % P_2 = P_2';

%pi/2 shifting of Preamble

P_3 = P_1.*repmat(exp(1i*pi*(0:3).'/2),size(P_1,1)/4,1);

c = repmat(exp(1i*pi*(0:3).'/2),size(P_1,1)/4,1);

%Upsample of  sequence by 3
P_4 = upsample(P_3, 3);

% q = interp(P_3,3);

%Resampling and filtering
Hn = [-1,0,1,1,-2,-3,0,5,5,-3,-9,-4,10,14,-1,-20,-16,14,33,9,-35,-42,11,64,40,-50,-96,-15,120,126,-62,-256,...
-148,360,985,1267,985,360,-148,-256,-62,126,120,-15,-96,-50,40,64,11,-42,-35,9,33,14,-16,-20,-1,14,10,...
-4,-9,-3,5,5,0,-3,-2,1,1,0,-1];

%Normalised filter weights
Hk_normal = ((sqrt(3)*Hn)./sqrt(sum(abs(Hn).^2)));

K = length(Hk_normal);

P_5 = filter(Hk_normal,1,[P_4; zeros((K-1)/2-1,size(P_3,2))]);

a = ([P_4; zeros((K-1)/2-1,size(P_3,2))]);


%Decimation of filtered Signal
P_6 = downsample(P_5((K-((K-1)/2)):end,:),2);
% r = decimate(P_5,2);
b = P_5((K-((K-1)/2)):end,:);


