function [x_points_e ,y_points_e ,z_points_e, range,cross_range] = SAR_process_mono(platform,radar,target, isac_waveform, waveform, coarse_mocomp_flag, std_dev)

% envelope values
radar.w_r=1;
radar.w_a =1;
NoOfTargets = length(target.zt);
% Squint angle (degrees)
theta_sq = 0;
TotalSwathTime = platform.AzimuthSwathTime + platform.SAT_mono;
t_slow = -TotalSwathTime/2:radar.PRI:TotalSwathTime/2;
t_sat = -platform.SAT_mono/2:radar.PRI:platform.SAT_mono/2;
% Range Chirp Rate (Hz/s)
Kr = radar.Bw/radar.Tp;
% Time Domain Sampling Interval (sec)
delt=1/(radar.Fs);

% Start time of sampling (sec)
Ts=((2*platform.Rc)-(2*target.R0))/radar.c;
% End time of sampling (sec)
Tf=(((2*platform.Rc)+(2*target.R0))/radar.c)+radar.Tp;% add pulse width
% Swath Samples
SwathSamples = round((Tf-Ts-radar.Tp)*radar.Fs);
% Generate the Tx Chirp
% Tx Chirp is generated for PW time interval starting from 0 to PW
% This chirp has to be delayed to get the final RawDataMatrix
PulseWidthSamples = round(radar.Tp*radar.Fs);
ChirpTimeSamples = -radar.Tp/2:delt:radar.Tp/2-delt;

% Transmitted signal

if waveform == 1
    % FMCW
    TxChirp = exp(+1i*pi*Kr.*ChirpTimeSamples.^2);
else 
    %ISAC Waveform
    TxChirp = isac_waveform(1:length(ChirpTimeSamples));
end

% Number of time (Range) samples) (SwathSamples + PulseWidthSamples)
rbins=round(((Tf-Ts))/delt);
% Fast time sampes corresponding to Swath + PulseWidth
t_fast=Ts+(0:rbins-1)*delt;

% Azimuth parameter
% Linear Azimuth FM rate
Ka=(2*platform.vc^2)./(radar.lambda*((platform.Rc)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MOTION ERRORS 
% Errors
% Translational motion errors
trans_err_flag = 1;
vy = platform.vc;
%Standard Deviation of Velocity of platform (along track - component) with error(m/s)
std_vy = std_dev;
%Mean Velocity of platform (cross track azimuth - component)(m/s)
vx=0;
%Standard Deviation of Velocity of platform (cross track azimuth - component) with error(m/s)
std_vx = std_dev;
%Mean Velocity of platform (cross track elevation - component)(m/s)
vz=0;
%Standard Velocity of platform (cross track elevation - component) with error(m/s)
std_vz = std_dev;

% Rotational Motion Errors
rot_err_flag = 0;
%Mean of Yaw angle (degrees)
yaw = 10;
% Standard Deviation of Yaw Angle
std_yaw = 0;
%Pitch angle (degrees)
pitch = 5;
% Standard Deviation of Pitch Angle
std_pitch = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MODELING THE RECCEIVED SIGNAL BY RADAR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RawData = zeros(length(t_slow),rbins);
s_rx    = zeros(length(t_slow),rbins);


%% Including motion errors
% Actual velocity of platform (while incorporating errors)
vy_e=  std_vy.*randn(length(t_slow),1) + vy; 
vx_e=  std_vx.*randn(length(t_slow),1) + vx; 
vz_e=  std_vz.*randn(length(t_slow),1) + vz; 

% Rotational motion errors
pitch_e= std_pitch.*ones(length(t_slow),1) + pitch;
yaw_e=  std_yaw.*ones(length(t_slow),1) + yaw;

% LOS unit vector
eta_los = [0; -sind(theta_sq); -cosd(theta_sq)];




 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
 % Trajectory followed by UAV 1 with no error
x_points = repmat(platform.xc,1,length(t_slow));
y_points = platform.yc+(platform.vc*t_slow);
z_points = repmat(platform.zc,1,length(t_slow));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Trajectory followed by UAV with error 
 x_points_e = platform.xc+(vx_e.'.*(t_slow));
 y_points_e = platform.yc+(vy_e.'.*(t_slow));
 z_points_e = platform.zc + (vz_e.'.*(t_slow));


 
%% Trajectory followed by UAV with no error

for tt = 1:NoOfTargets
    for j=1:length(t_slow)
        % Angle caused by rotational error with ideal
    beta = 0;
    % Direction vector caused by rotational error
    eta_yaw_pitch = [0 0 0];
    % Cross range shift caused by rotational error
    tgt_shift = 0;
  
        % ideal range of target with no motion error (r_mo)
        range_ideal(j) = sqrt(((target.zt(tt)-z_points(j))^2)+((target.xt(tt)-x_points(j))^2)+((target.yt(tt)-y_points(j))^2));% [x,y,z]
      
          range_wtme = range_ideal;
        tgt_shift = 0;

    if trans_err_flag == 1
    % Trajectory followed by UAV with error 
        range_wtme(j) =  sqrt(((target.zt(tt)-z_points_e(j))^2)+((target.xt(tt)-x_points_e(j))^2)+((target.yt(tt)-y_points_e(j))^2));
    end
     if rot_err_flag == 1  
        eta_yaw_pitch = [(deg2rad(yaw_e(j))*sind(theta_sq))-(deg2rad(pitch_e(j))*cosd(theta_sq));-sind(theta_sq)-(deg2rad(yaw_e(j))*deg2rad(pitch_e(j))*cosd(theta_sq));-cosd(theta_sq)];
        % squint angle (radians)
        beta = abs(acos(dot(eta_los,eta_yaw_pitch)));
        tgt_shift = tan(beta)*range_ideal;
    end
        % range of target with translationa and rotational motion error 
       range_wtrme(j) = sqrt(range_wtme(j)^2+tgt_shift^2);
     % Range error for coarse motion compensation
         del_R_all(j) = (range_wtrme(j)-range_ideal(j) )*cosd(beta); 
        % Delay in fast time samples reference with respect to center range for
        % proper placement of LFM in RawData Matrix

        % Robust Indexing Calculation
        td(j) = (2*range_wtrme(j))/radar.c - ((2*platform.Rc)/radar.c);
        td_samples = round(td(j)*(1/delt) + SwathSamples/2);
        
        start_idx = td_samples;
        end_idx   = td_samples + PulseWidthSamples - 1;
        
        % THE FIX: Only attempt to write if the pulse is within the simulated window
        if start_idx >= 1 && end_idx <= rbins
            s_rx(j, start_idx:end_idx) = target.rcs_lin(tt) .* radar.w_r .* radar.w_a .* ...
                TxChirp .* exp(-1j*(2*pi*radar.fc*radar.ic*(2*range_wtrme(j))));
        else
            % If it's out of bounds, we simply don't record this pulse.
            % This prevents the crash and simulates the signal being lost.
            % fprintf('Warning: Signal out of bounds at slow-time index %d\n', j);
        end

       
         %td(j) = (2*range_wtrme(j))/radar.c-(((2*platform.Rc))/radar.c);
         %td_samples = round(round(td(j)*(1/delt))+SwathSamples/2);
        % Final received signal for real trajectory
        
        %s_rx(j,td_samples:td_samples+PulseWidthSamples-1)= target.rcs_lin(tt).*radar.w_r.*radar.w_a.*TxChirp.*exp(-1j*(2*pi*radar.fc*radar.ic*(2*range_wtrme(j))));
    
    end
    RawData = RawData+s_rx;
    % disp(tt)
end


% Add noise
s_rx_noise = RawData;%awgn(s_rx,SNR);

% course motion compensation
    if coarse_mocomp_flag == 1
        for n = 1:(length(t_slow))
        s_rx_noise(n,:) = s_rx_noise(n,:).*exp(1j*4*pi*radar.fc*radar.ic*(del_R_all(n)));
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIGNAL PROCESSING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Range Reference Signal
s_ref = TxChirp; % This is the pulsewidth of the transmitted LFM

% Reference Signal in frequency domain
FFT_Size = size(s_rx_noise,2);
s_ref_freq = conj(fft(s_ref,FFT_Size));
s_rx_freq = zeros(length(t_slow),rbins);

% Range Compression
for k=1:(length(t_slow))
    s_rx_freq(k,:) = fft(s_rx_noise(k,:),FFT_Size); % Range FFT
    s_mf_freq(k,:) = s_rx_freq(k,:).*s_ref_freq;% Range Matched Filtering
    s_mf_time_Temp(k,:) = ifft(s_mf_freq(k,:)); % Range IFFT [s_mf_time(k,1:)]
    s_mf_time(k,:) = s_mf_time_Temp(k,1:SwathSamples);
end
%xaxis = linspace(-SwathSamples/2,SwathSamples/2,size(s_mf_time,2));
% figure;imagesc(abs(s_mf_time));

%% Range Cell Migration Correction (RCMC)
% rcmc=s_mf_time;
%% Range Cell Migration Correction (RCMC)
%% Kindly mention the reference source for this algorithm
deltaR=(1*radar.lambda^2*(platform.Rc).*((Ka)*(t_slow)).^2)/(8*platform.vc^2);
range_res = (radar.c*delt)/2;
cells=round(deltaR/range_res);
rcm_max= max(cells);

rcmc=s_mf_time;
 for k=1:length(t_slow)

        rcmc(k,:)=circshift(s_mf_time(k,:),[0, -cells(k)]);

end

% Azimuth FFT
for l=1:size(rcmc,2)
    rcmc(:,l)=fft(rcmc(:,l));
end
%% Azimuth Compression
% Azimuth Reference Signal
s_ref_azi_time=exp(-1j*pi*(Ka).*((t_sat)).^2);

% Azimuth Matched Filter Spectrum
N_FFT = size(rcmc,1);
s_ref_azi_freq = conj(fft(s_ref_azi_time,N_FFT));
s_ref_azi_freq = s_ref_azi_freq.';
% Azimuth Compression
for p=1:SwathSamples
    sar_freq(:,p)= rcmc(:,p).*s_ref_azi_freq;
    % Azimuth IFFT
    sar_time(:,p) = ifft(sar_freq(:,p));
end
sar_time = sar_time(1:platform.AzimuthSamples,:);

 %% Defining axis

%Cross range axis (m)
azi_axis = linspace(-size(sar_time,1)/2,size(sar_time,1)/2,size(sar_time,1)).*(platform.Delta_X);
% Range axis (m)
ground_range_axis = linspace(-size(sar_time,2)/2,size(sar_time,2)/2,size(sar_time,2)).*(radar.c/(2*radar.Fs));


%% Adding land clutter
% avg_vel_wind = 20;
% [land_clutter] = generate_landclutter(avg_vel_wind,radar,ground_range_axis,azi_axis);
% 
% y_out_land_clutter = (sar_time);%+ (land_clutter).';
% 
% % Conveting back to time domain
% for pp=1:SwathSamples
%     % Azimuth IFFT
%     sar_time_2(:,pp) = ifft(y_out_land_clutter(:,pp));
% end
    sar_time_2 = sar_time;%sar_time_2(1:platform.AzimuthSamples,:);% final sar data
%% Final Plotting (Kindly check this portion ofthe code )
 %% Defining axis

%Cross range axis (m)
azi_axis = linspace(-size(sar_time_2,1)/2,size(sar_time_2,1)/2,size(sar_time_2,1)).*(platform.Delta_X);
% Range axis (m)
ground_range_axis = linspace(-size(sar_time_2,2)/2,size(sar_time_2,2)/2,size(sar_time_2,2)).*(radar.c/(2*radar.Fs));

sar_2d =20*log10((1/platform.AzimuthSamples)*(1/SwathSamples)*(abs(sar_time_2)));
cmax = max(max(sar_2d)); 
range_axis = t_fast.*radar.c.*0.5;
range_axis = range_axis(1:SwathSamples);
% figure();
% imagesc(range_axis,azi_axis,sar_2d,[cmax-30 cmax]); colorbar;
% xlabel('Range (m)'), ylabel('Azimuth (m)')
% set(gca, 'YDir', 'normal');
% colormap(jet(256));

 %% Find peak corresponding to target
 % iterationData = 5; % Total number of peaks
 % for k = 1: iterationData
      [cross_range_index range_index] = find(sar_2d==cmax);% this command will change
 %     cross_range = azi_axis(cross_range_index);
 %     range = range_axis(range_index);
 %     cr_tgt(:,k) =   cross_range;
 %     r_tgt(:,k) =   range;
 % end
%  Cross range estimate of target
 cross_range = azi_axis(cross_range_index);
 % Range estimate of target
% range = ground_range_axis(range_index);
 range = range_axis(range_index);
% fprintf('Ground truth range of target %d m \n',xt);
% fprintf('Ground truth range of target %d m \n',((range_ideal(600))));
% fprintf('Ground truth cross range of target %d m \n',target.yt(1));
% fprintf('Measured range of target %d m \n',range);
% fprintf('Measured cross range of target %d m \n',cross_range);
end
