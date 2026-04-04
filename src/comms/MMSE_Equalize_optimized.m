function [y, CSI] = MMSE_Equalize_optimized(x, chanEst,noiseVarEst)

% Perform equalization
numSym = size(x,2);
CSI = zeros(size(x, 1), numSym,size(x,3)); % Pre-allocation here for code generation
 % SISO
    chanEstSISO = chanEst; % For codegen
    CSI = real(chanEstSISO.*conj(chanEstSISO)) + noiseVarEst;
    y = x.*(conj(chanEstSISO)./CSI);
      
end