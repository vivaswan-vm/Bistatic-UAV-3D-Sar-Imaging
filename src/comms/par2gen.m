  function G = par2gen(H)
[parityCheckLen,blockLength] = size(H);
infolen = blockLength - parityCheckLen;
H1 = H(:,1:infolen);
H2 = H(:,(infolen+1):end);

% Do LU factorization on H2
[PL, PB, rowOrder, invertible] = gf2factorize(H2);
if invertible % Should always be invertible for LDPC codes supported in 802.11
    PB = PB(rowOrder, :);
end

pbits = false(parityCheckLen,infolen);

for i = 1:parityCheckLen
    PL(i,i) = 0;
    PB(i,i) = 0;
end

for i = 1:infolen
    r = gf2solve(PL,H1(:,i),1);                 % Matrix multiply
    pbits(:,i) = gf2solve(PB,r(rowOrder),-1);   % Backward substitution
end

G = [eye(infolen) double(pbits')];


function [A, B, chosen_pivots, invertible] = gf2factorize(X)
% Factorize a square matrix in GF(2).

n = size(X,1);

A = logical(eye(n,n));
B = full(X~=0);
chosen_pivots = zeros(n,1); % Haven't chosen any pivots yet.
invertible = true; % Assume that X is invertible in GF(2).

for col = 1:n
  candidate_rows = B(:,col);
  candidate_rows(chosen_pivots(1:(col-1))) = 0; % Never use a chosen pivot.
  candidate_rows_ind = find(candidate_rows); % Convert into row indices.
  
  if isempty(candidate_rows_ind)
    invertible = false; % X is not invertible in GF(2).
    break;
  else
    pivot = candidate_rows_ind(1);  % Choose the first candidate row as the pivot row.
    chosen_pivots(col) = pivot;     % Record this pivot.
    
    % Find all nonzero elements in the pivot row.
    % They will be xor-ed with the corresponding elements in other
    % candidate rows.
    columnind = find(B(pivot,:));
    
    % Subtract the pivot row from all other candidate_rows.
    % Take advantage of the fact that we are working in GF(2).
    % Just use logical NOT.
    % As we continue in this loop, B will become "psychologically"
    % upper triangular.
    for k = 1:length(columnind)
        for j = 2:length(candidate_rows_ind)
            B(candidate_rows_ind(j), columnind(k)) = not(B(candidate_rows_ind(j), columnind(k)));
        end
    end
    
    % Update the lower triangular matrix A.
    for j = 2:length(candidate_rows_ind)
        A(candidate_rows_ind(j), pivot) = 1;
    end
  end
end


function y = gf2solve(A, y, direction)
if direction == 1
    columnindex = 1; % Start from the first column for forward substitution
else
    columnindex = length(y); % Start from the last column for backward substitution
end

columncounter = 1;
while columncounter < length(y)
    if y(columnindex)
        y(:) = xor(y,A(:,columnindex));
    end
    
    columncounter = columncounter + 1;
    columnindex = columnindex + direction;
end

% Following code is used to generate the mat files for the encoder. The G
% matrix is reshaped into group of 16 bits, which is converted into decimal
% and stored as unsigned integer in ldpcMatrices.mat file.

% rate = '1/2';
% blockLength = 648;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_1_2_648 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '1/2';
% blockLength = 1296;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_1_2_1296 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '1/2';
% blockLength = 1944;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_1_2_1944 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '2/3';
% blockLength = 648;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_2_3_648 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '2/3';
% blockLength = 1296;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_2_3_1296 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '2/3';
% blockLength = 1944;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_2_3_1944 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '3/4';
% blockLength = 648;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_3_4_648 = uint16(bi2de(reshape(outg,[],12)));
% 
% rate = '3/4';
% blockLength = 1296;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_3_4_1296 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '3/4';
% blockLength = 1944;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_3_4_1944 = uint16(bi2de(reshape(outg,[],12)));
% 
% rate = '5/6';
% blockLength = 648;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_5_6_648 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '5/6';
% blockLength = 1296;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_5_6_1296 = uint16(bi2de(reshape(outg,[],16)));
% 
% rate = '5/6';
% blockLength = 1944;
% [~,g] = wlan.internal.ldpcMatrix(rate,blockLength);
% sizeG = size(g);
% outg = g(:,sizeG(1)+1:end).';
% ldpcMatrices.PT_5_6_1944 = uint16(bi2de(reshape(outg,[],16)));
% 
% save('ldpcMatrices.mat','-struct', 'ldpcMatrices')