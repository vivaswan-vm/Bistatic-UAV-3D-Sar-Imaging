function pl = get_transmitted_pilots(NCW)


%%%%% Pilots
P_seq = [-1; 1; -1; 1; 1; -1; -1; -1; -1 ;-1; 1; 1; 1; -1; 1; 1];
pilot = double(wlanScramble(zeros(NCW+1,1,'int8'),ones(7,1,'int8')).');
polaritySeq = (2*pilot(1+1:end)-1);
% Pilot symbols 16 x Nsym
pl = bsxfun(@times,polaritySeq,P_seq);


end