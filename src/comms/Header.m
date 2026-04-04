%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Header Block for 802.11ad
%Soumya Jain
%13/8/21
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [H_1,H_f] = Header(data_size_bytes)
% clc
% clear all;
% close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%fprintf('Taking Scrambling Initialisation Value as 2\n');
%data_size_bytes = str2num(input('\n','s'));
%cp length
cpLen = 128;

%Training Length
trnlen = 0;

%Number of parity bits 
parity = 16;

%Number of IFFT points
IFFTN = 512;

%Exapnsion Factor
exp_factor = 42;

%Base Matrix
rotmatrix = ...
    [35 19 41 22 40 41 39 6 28 18 17 3 28 -1 -1 -1;
     29 30 0 8 33 22 17 4 27 28 20 27 24 23 -1 -1;
     37 31 18 23 11 21 6 20 32 9 12 29 -1 0 13 -1;
     25 22 4 34 31 3 14 15 4 -1 14 18 13 13 22 24];

%Q Matrix to be multiplied during QPSK Modulation
Q=1/sqrt(5).*[1 2; -2 1]; 

%Pilot Subcarriers
%p=[-1 1 -1 1 1 -1 -1 -1 -1 -1 1 1 1 -1 1 1]; 
p=[1 -1 1 -1 -1 1 1 1 1 1 -1 -1 -1 1 -1 -1];
%Header block fields

%Scrambler Initialisation (7 bits)
scramInit =flip(logical([0;0;0;0;0;1;0]));

%Modulation and Coding Scheme (MCS =17) (5 bits)
MCS = ([1; 0; 0; 0; 1]);

%Lenght of data(in Bytes) (18 bits)
len = (de2bi(data_size_bytes,18));
len_byte = (len');

%Additional PPDU (1 bit)
appdu =(0);

%Packet type(TRN -T) Bit value = 0 (1 bit)
pck_type = (0);

%Training Lenght (5 bits)
trn = ([de2bi(trnlen/4,5)]);
trn_len = (trn)';

%Aggregation (1 bit)
agg = 0;

%Beam Tracking request (1 bit) % used only with TRN-R
btr = 0;

%Tone pairing request (1 bit)
tpr = 0;

%DTP Indicator (1 bit)
dtpind = 0;

%Last RSSI (4 bits)
lastrssi = [0;0;0;0];

%Turnaround (1 bit)
ta = 0;

%Reserved (2 bits)
r = [0;0];

%HCS (16 bits)
hcs = zeros(16,1);

%48 bits of the header
Header_48bits =([scramInit;MCS;len_byte;appdu;pck_type;trn_len;agg;btr;tpr;dtpind;lastrssi;ta;r]);

%Generating CRC bits
crcgenerator = wlanCRCGenerate(Header_48bits,16);

%Final 64 bits of header
Header_64bits = [Header_48bits; crcgenerator];
%Scrambling 
sc = 2;

Header = [MCS;len_byte;appdu;pck_type;trn_len;agg;btr;tpr;dtpind;lastrssi;ta;r;crcgenerator];
y = wlanScramble(Header,sc);

%Scrambled Sequence
H_1 = [scramInit; y];

%Zero Padding of header bits
H_2 = [H_1; zeros(440,1)];

%LDPC Encoding
H_3 = [H_2; zeros(parity,1)];

parity_chk = ldpc_b(rotmatrix,exp_factor);

G = par2gen(parity_chk);

sizeG = size(G);
PT = double(G(:,sizeG(1)+1:end)).';
parity_bits = PT*double(H_2);

parity_bits_mod2 = mod(parity_bits,2);

%Generation of 672 bit sequence
x2 = [H_1; parity_bits_mod2(1:160)];

x1 = [H_1; parity_bits_mod2(1:84); parity_bits_mod2(93:168)];

c1 = [H_1; parity_bits_mod2(9:168)];

x = [x1;x2];

%Scrambling of header sequence; Initialisation sequence is 127 or 1111111
init = 127;

pad_sequence = wlanScramble(x,init);

%Final Header Sequence after LDPC Encoding
H_4 = [c1; pad_sequence];

%QPSK Modulation
% qpskMod = comm.QPSKModulator('BitInput',true);
% H_5 = qpskMod(H_4);

H_5 = wlanConstellationMap(reshape(H_4,2,numel(H_4)/2).',2);

% for j=1:2:length(H_5)
%         y=H_5(j:j+1);
%         H_6(j:j+1)=Q*y;
% end
 H_6 = (Q*[H_5(:,1) H_5(:,2)].').';
 H_6 = (reshape(H_6, numel(H_6),1));
% H_6 = H_6';
% figure(3)
% plot(abs(H_6));

%Subcarrier Mapping
H_7=[zeros(79,1); H_6(1:27); p(1); H_6(28:46); p(2); H_6(47:65); p(3); H_6(66:84);...
     p(4); H_6(85:103); p(5) ;H_6(104:122); p(6); H_6(123:141); p(7); H_6(142:160);...
     p(8); H_6(161:168); 0; 0; 0; H_6(169:176); p(9); H_6(177:195); p(10); H_6(196:214); 
     p(11); H_6(215:233); p(12); H_6(234:252); p(13); H_6(253:271); p(14); H_6(272:290); ...
     p(15); H_6(291:309); p(16); H_6(310:336); zeros(78,1)];

%512 Point IFFT
H_8 = ifft(ifftshift(H_7));

%CP Addition
H_9 = [H_8(385:512); H_8];
yout = reshape(H_9,[(IFFTN+cpLen(1))*1 1]);
%Amplification factor
%Number of active sub carriers (sub carriers excluding guard band and DC)
%NULL Subcarriers
%Active_sub = 336+16; Total subcarriers + pilot sub carriers
%A = IFFTN/(sqrt(Active_sub));
H_f = yout.*27.2897;  %%Final Header
 
 
%  figure(1)
% plot(abs(f));
% xlabel('Time Samples');
% ylabel('Amplitude (Volts)');
% title('Generated Output');

end

