%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%WIndowing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [wlan_f] = windowing(data,data_size_bytes)
%Frame Generation
[header,data] = Data_msym(data, data_size_bytes);
datapacket = [header;data];
preamble = Preamble_part();
frame = [preamble;datapacket];
%Size of frame
fsize = length(frame);

%Ending Index of Preamble
p_ind = length(preamble);

%OFDM Symbol Length
s_len = 640;
%Windowing length
w_len = 16;
b = w_len/2;
W = w_len-1;
aLen = 0;
%Guard Interval Length 
GI = 128;

%Indices for Windowing
pre_in = -w_len/2+1:w_len/2-1;
mid = w_len/2:(s_len-w_len/2)-1;
post = s_len-w_len/2:(s_len+w_len/2)-1;
ind = [pre_in mid post].';

%Windowing Magnitude
pre_m = sin(pi/2*(0.5+(pre_in)/(w_len))).^2;
mid_m = ones(1,length(mid)); 
post_m = sin(pi/2*(0.5-((post)-s_len)/w_len)).^2; 
mag = [pre_m mid_m post_m].';

%Header and Data 
w1 = frame(p_ind+1:end);

%Reshaping into 2 columns
w2 = reshape(w1, s_len,(length(w1)/s_len));

for i = 1:(length(w1)/s_len)
    w3(:,i) =[w2((end-(abs(ind(1))+GI)+1:end-GI),i); w2(:,i); w2((GI+(1:w_len/2)),i)];
    
    w4(:,i) = w3(:,i).*mag;
   
end
 %Overlaping window region
   
  firstRegion = w4(1:end-W,1,:);
  overlapRegion = w4(end-W+1:end,1:end-1,:)+w4(1:W,2:end,:);
  middleRegion = [overlapRegion; w4(W+1:end-W,2:end,:)];
  lastRegion = w4(end-W+1:end,end,:);
  w5 = [firstRegion;reshape(middleRegion,[],1,size(w4,3));lastRegion];
  
  %Prefix to ofdm symbol
  p = w5(1:w_len/2-1,:);
  
  %Preamble Samples
  pre = frame(p_ind-w_len/2+2:p_ind);
  
  %preamble overlapping
  p_o = p + pre;
  
  %final frame
  w6 = [frame(1:p_ind-(w_len/2)+1);p_o;w5(w_len/2:end)]; %%Windowed Packet
  wlan_f = w6(1:fsize); %%Txframe
  
  %figure(1)
  %plot(abs(wlan_f));
  
  temp = w6(end-(b)+1:end,:);
  wlan_f(1:b) = wlan_f(1:b)+ temp(1:b);
  wlan_f(end+1:end,:) = wlan_f(end+1:end,:)+w6(1:aLen,:);
  
end


