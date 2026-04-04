% This work is based on the paper:
% T. Richardson and R. Urbanke. "Efficient Encoding of Low-Density
% Parity-Check Codes". IEEE Trans. on Information theory, Feb. 2001

% *******************************************************
% Taken from IEEE 802.11n: HT LDPC matrix definitions
% You can change this according to your needs :)
function [H]= ldpc_b(rotmatrix,Z)

H = zeros(size(rotmatrix)*Z);
Zh = diag(ones(1,Z),0);

% Convert into binary matrix
for r=1:size(rotmatrix,1)
    for c=1:size(rotmatrix,2)
        rotidx = rotmatrix(r,c);
        if (rotidx > -1)            
            Zt = circshift(Zh,[0 rotidx]);
        else
            Zt = zeros(Z);
        end
        limR = (r-1)*Z+1:r*Z;
        limC = (c-1)*Z+1:c*Z;
        H(limR,limC) = Zt;
    end
end


