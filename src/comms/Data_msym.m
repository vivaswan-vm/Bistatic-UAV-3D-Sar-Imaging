% clc;
% clear all;
% close all
function [f,d_13] = Data_msym(data, data_size_bytes)

%Header bits
[Head,f] = Header(data_size_bytes);
%%%%%%%%%% Data Generation
% d_1 = zeros(960,1);
% for i = 2:20
%    if(mod(i,2)==0)
%        d_1((i-1)*48 : ((i*48)-1)) = zeros(48,1);      
%    else
%        d_1((i-1)*48 : ((i*48)-1)) = ones(48,1);
%    end
% end
% d_2=[d_1;zeros(48,1)];

d_2 = data;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% Parameters
scramInit = flip(logical([0;0;0;0;0;1;0]));

LCW = 672;
R = 3/4;
LCWD = R*672;

NCW = length(d_2)/LCWD;
Nfft=512;
NSD = LCW/2;
CPlen=128;
Z=42;              %%% Expansion Factor 
rotmatrix = ...
    [35 19 41 22 40 41 39 6 28 18 17 3 28 -1 -1 -1;
     29 30 0 8 33 22 17 4 27 28 20 27 24 23 -1 -1;
     37 31 18 23 11 21 6 20 32 9 12 29 -1 0 13 -1;
     25 22 4 34 31 3 14 15 4 -1 14 18 13 13 22 24];

Q=1/sqrt(5).*[1 2; -2 1]; 

%%%%% Pilots
P_seq = [-1; 1; -1; 1; 1; -1; -1; -1; -1 ;-1; 1; 1; 1; -1; 1; 1];
pilot = double(wlanScramble(zeros(NCW+1,1,'int8'),ones(7,1,'int8')).');
polaritySeq = (2*pilot(1+1:end)-1);
pl = bsxfun(@times,polaritySeq,P_seq);


x = [Head(8:end)];
%%%%%%%%% Scrambler

abits = [x;d_2];
scram= wlanScramble([x;d_2],2);

%Scrambled Data bits
d_3 = scram(58:end);


%Reshaping The matrix
d_4 = reshape(d_3,LCWD,NCW);
%%%%%%%%%%% LDPC encoding
parity=zeros(168,1);

H = ldpc_b(rotmatrix,Z);
G = par2gen(H); 

sizeG = size(G);
PT = double(G(:,sizeG(1)+1:end)).';
% PT1 = zeros(168,515);
% PT2 = [PT1 PT]
for j = 1:NCW
    P(:,j) = PT*double(d_4(:,j));    %%%%%parity generated
    P1(:,j)=mod(P(:,j),2);  
    d_5(:,j)=[d_4(:,j); P1(:,j)];

end

d_6 = reshape(d_5,(LCW*NCW),1);

%QPSK Modulation
c_map = wlanConstellationMap(reshape(d_6,2,numel(d_6)/2).',2);

Q = (1/sqrt(5))*[1 2; -2 1]; % DCM mapping matrix
d_7 = (Q*[c_map(:,1) c_map(:,2)].').';
%%%%%%%%%%%%%%%%%%%%%%%%5Verify from here%%%%%%%%%%%%%
d_8_1 = reshape(d_7(:,1),NSD/2,NCW);

d_8_2 = reshape(d_7(:,2),NSD/2,NCW);

d_8 = [d_8_1; d_8_2];

for i = 1:NCW
    d_9(:,i)=[zeros(79,1); d_8(1:27,i); pl(1,i); d_8(28:46,i); pl(2,i); d_8(47:65,i); pl(3,i); d_8(66:84,i);...
     pl(4,i); d_8(85:103,i); pl(5,i); d_8(104:122,i); pl(6,i); d_8(123:141,i); pl(7,i); d_8(142:160,i);...
     pl(8,i); d_8(161:168,i); 0; 0; 0; d_8(169:176,i); pl(9,i); d_8(177:195,i); pl(10,i); d_8(196:214,i); 
     pl(11,i); d_8(215:233,i); pl(12,i); d_8(234:252,i); pl(13,i); d_8(253:271,i); pl(14,i); d_8(272:290,i); ...
     pl(15,i); d_8(291:309,i); pl(16,i); d_8(310:336,i); zeros(78,1)];
 
 %%%IFFT
 d_10 (:,i)= ifft(ifftshift(d_9(:,i)));
 
 %%%%%CP Addition
 d_11(:,i)= [d_10(385:512,i); d_10(:,i)];
end
d_12 = reshape(d_11, numel(d_11),1);

%%%%%Normalisation Factor
Active_sub = 336+16; %Total subcarriers + pilot sub carriers
A = Nfft/(sqrt(Active_sub));
d_13 = d_12.*A;

end
