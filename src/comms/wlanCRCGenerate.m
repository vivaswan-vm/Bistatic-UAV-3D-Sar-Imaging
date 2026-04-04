function y = wlanCRCGenerate(x,varargin)
%wlanCRCGenerate Generate CRC checksum
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
% 
%   Y = wlanCRCGenerate(X) generates CRC checksum for an input message X as
%   per IEEE Std 802.11-2012 Section 20.3.9.4.4 and IEEE 802.11ad 2012
%   Sections 21.3.7.
%   
%   Y is an int8 column vector of length 8 containing the checksum.
%  
%   X is an binary column vector containing the message bits.
%
%   Y = wlanCRCGenerate(X,NUMBITS) generates CRC checksum with a specified
%   number of bits. When NUMBITS is 4, the CRC is generated as described in
%   IEEE P802.11ah/D5.0 Section 24.3.8.2.1.5. When NUMBITS is 16, the CRC
%   is generated as described in IEEE 802.11ad-2012 Section 21.3.7.
%   Otherwise, the CRC is generated as described in IEEE 802.11-2012
%   Section 20.3.9.4.4.
%
%   % Example 1: Generate the CRC for the HT-SIG field according to IEEE
%   % Std 802.11-2012 Section 20.3.9.4.4.
%
%     m = [1 1 1 1 0 0 0 1 0 0 1 0 0 1 0 0 1 1 0 0 0 0 0 0].';
%     y = wlan.internal.wlanCRCGenerate(m);
%
%   % Example 2: Generate the CRC for the S1G-SIG field according to IEEE
%   % P802.11ah/D5.0 Section 24.3.8.2.1.5.
%
%     m = [1 1 1 1 0 0 0 1 0 0 1 0 0 1 0 0 1 1 0 0 0 0 0 0].';
%     numBits = 4;
%     y = wlan.internal.wlanCRCGenerate(m,numBits);
%
%   See also wlanCRCDetect.

%   Copyright 2015-2017 The MathWorks, Inc.

%#codegen

narginchk(1,2)

if nargin>1
    % Allow number of bits and be specified
    numBits = int16(varargin{1});
    shiftReg = uint16(2^varargin{1}-1);
    finalXOR = uint16(2^varargin{1}-1);
else
    % Section 20.3.9.4.4 in IEEE Std 802.11-2012 
    numBits  = int16(8);
    shiftReg = uint16(255);
    finalXOR = uint16(255);
end

% Generator polynomial selected based on number of CRC bits
if numBits==4
    genPoly = uint16(3);  % Section 24.3.8.2.1.5 IEEE P802.11ah/D5.0 
elseif numBits==8
    genPoly = uint16(7);  % Section 20.3.9.4.4 in IEEE Std 802.11-2012 
else % numBits==16
    genPoly = uint16(2^12+2^5+2^0); % Section 21.3.7 in IEEE Std 802.11ad-2012 
end

for i = 1:length(x)
    regOut = bitand(bitshift(shiftReg, 1 - numBits), uint16(1));
    regIn  = bitxor(regOut, uint16(x(i)));

    if regIn
        shiftReg = bitshift(bitxor(bitshift(genPoly, -1), shiftReg), 1);
        shiftReg = shiftReg + 1;
    else
        shiftReg = bitshift(shiftReg, 1);
    end
end  

shiftReg = bitxor(shiftReg, finalXOR);         

y = coder.nullcopy(zeros(numBits, 1, 'int8'));
for i = 1:numBits
    y(i) = bitand(bitshift(shiftReg, -(numBits - i)), 1);
end

end