function [range,cross_range] = SAR_process_bi(x_points_e ,y_points_e ,z_points_e,struc_constants,struc_trajectory,radar, platform, target, isac_waveform, waveform, coarse_mocomp_flag, std_dev)

TotalSwathTime = platform.AzimuthSwathTime + platform.SAT_bi;
t_slow = -TotalSwathTime/2:radar.PRI:TotalSwathTime/2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% envelope values
radar.w_r=1;
radar.w_a =1;
NoOfTargets = length(target.zt);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MOTION ERRORS for bistatic
% Errors
% Translational motion errors
trans_err_flag = 0;
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

%% Including motion errors
% Actual velocity of platform (while incorporating errors)
vy_e=  std_vy.*randn(length(t_slow),1) + vy; 
vx_e=  std_vx.*randn(length(t_slow),1) + vx; 
vz_e=  std_vz.*randn(length(t_slow),1) + vz; 

% Rotational motion errors
pitch_e= std_pitch.*ones(length(t_slow),1) + pitch;
yaw_e=  std_yaw.*ones(length(t_slow),1) + yaw;

% LOS unit vector
eta_los = [0; -sind(platform.theta_sq); -cosd(platform.theta_sq)];

err_x = 0.2;
err_y = 0.1;
err_z = 0.25;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%----------------------------------------------------------------------------------------------
for i=1:length(t_slow)
if i<=length(x_points_e)
[radar,platform] = CommData_function(t_slow(i),x_points_e(i) ,y_points_e(i) ,z_points_e(i),struc_constants, struc_trajectory, radar, platform, target);

 % Trajectory followed by UAV1 with error 
 x_points_e1(i) = platform.x_points_e_ext;%platform.xc+(vx_e.'.*(t_slow));
 y_points_e1(i) = platform.y_points_e_ext;
 z_points_e1(i) = platform.z_points_e_ext;

else i > length(x_points_e)
   % Trajectory followed by UAV1 with error 
 x_points_e1(i) = platform.xc+(vx_e(i)*(t_slow(i)));
 y_points_e1(i) = platform.yc+(vy_e(i)*(t_slow(i)));
 z_points_e1(i) = platform.zc+(vz_e(i)*(t_slow(i)));
end
end
%% ---------------------------------------------------------------------------------------------

% Derived Parameters
t_sat = -platform.SAT_bi_ext/2:radar.PRI_ext:platform.SAT_bi_ext/2;
% Range Chirp Rate (Hz/s)
Kr = radar.Bw_ext/radar.Tp_ext;
% Time Domain Sampling Interval (sec)
delt=1/(radar.Fs_ext);
% Start time of sampling (sec)
Ts=((platform.Rc_ext+platform.Rc_2_ext)-(2*target.R0))/radar.c;
% End time of sampling (sec)
Tf=(((platform.Rc_ext+platform.Rc_2_ext)+(2*target.R0))/radar.c)+radar.Tp_ext;% add pulse width
% Swath Samples
SwathSamples = round((Tf-Ts-radar.Tp_ext)*radar.Fs_ext);
% Generate the Tx Chirp
% Tx Chirp is generated for PW time interval starting from 0 to PW
% This chirp has to be delayed to get the final RawDataMatrix
PulseWidthSamples = round(radar.Tp_ext*radar.Fs_ext);
ChirpTimeSamples = -radar.Tp_ext/2:delt:radar.Tp_ext/2-delt;

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
Ka=(2*platform.vc_ext^2)./(radar.lambda_ext*(0.5*(platform.Rc_ext+platform.Rc_2_ext)));
% MODELING THE RECCEIVED SIGNAL BY RADAR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RawData = zeros(length(t_slow),rbins);
s_rx    = zeros(length(t_slow),rbins);

% Trajectory followed by UAV 1 with no error
x_points = repmat(platform.xc_ext,1,length(t_slow));
y_points = platform.yc_ext+(platform.vc_ext*t_slow);
z_points = repmat(platform.zc_ext,1,length(t_slow));

% Trajectory followed by UAV 2 with no error
x_points_2 = repmat(platform.xc_2_ext,1,length(t_slow));
y_points_2 = platform.yc_ext+(platform.vc_ext*t_slow);
z_points_2 = repmat(platform.zc_2_ext,1,length(t_slow));


 % Trajectory followed by UAV2 with error 
 x_points_e2 = platform.xc_2_ext+((vx_e.'+err_x).*(t_slow));
 y_points_e2 = platform.yc_ext+((vy_e.'+err_y).*(t_slow));
 z_points_e2 = platform.zc_2_ext + ((vz_e.'+err_z).*(t_slow));
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%% Trajectory followed by UAV with no error

for tt = 1:NoOfTargets
    for j=1:length(t_slow)

        beta = 0;
    % Direction vector caused by rotational error
    eta_yaw_pitch = [0 0 0];
    % Cross range shift caused by rotational error
    tgt_shift = 0;
        % ideal range of target with no motion error (r_mo)
        range_ideal(j) = sqrt(((target.zt(tt)-z_points(j))^2)+((target.xt(tt)-x_points(j))^2)+((target.yt(tt)-y_points(j))^2));% [x,y,z]
        range_ideal_2(j) = sqrt(((target.zt(tt)-z_points_2(j))^2)+((target.xt(tt)-x_points_2(j))^2)+((target.yt(tt)-y_points_2(j))^2));
        % Delay in fast time samples reference with respect to center range for
        % proper placement of LFM in RawData Matrix
        % td(j) = 2*(range_ideal(j))/c-2*Rc/c;
        % td_samples = round(round(td(j)*(1/delt))+SwathSamples/2);
          range_wtme1 = range_ideal;
          range_wtme2 = range_ideal_2;
        tgt_shift = 0;

    if trans_err_flag == 1
    % Trajectory followed by UAV1 with error 
        range_wtme1(j) =  sqrt(((target.zt(tt)-z_points_e1(j))^2)+((target.xt(tt)-x_points_e1(j))^2)+((target.yt(tt)-y_points_e1(j))^2));
        range_wtme2(j) =  sqrt(((target.zt(tt)-z_points_e2(j))^2)+((target.xt(tt)-x_points_e2(j))^2)+((target.yt(tt)-y_points_e2(j))^2));
    end
     if rot_err_flag == 1  
        eta_yaw_pitch = [(deg2rad(yaw_e(j))*sind(platform.theta_sq))-(deg2rad(pitch_e(j))*cosd(platform.theta_sq));-sind(platform.theta_sq)-(deg2rad(yaw_e(j))*deg2rad(pitch_e(j))*cosd(platform.theta_sq));-cosd(platform.theta_sq)];
        % squint angle (radians)
        beta = abs(acos(dot(eta_los,eta_yaw_pitch)));
        tgt_shift = tan(beta)*range_ideal;
    end
       % range of target with translational and rotational motion error 
       range_wtrme1(j) = sqrt(range_wtme1(j)^2+tgt_shift^2);
       range_wtrme2(j) = sqrt(range_wtme2(j)^2+tgt_shift^2);
     % Range error for coarse motion compensation
         del_R_all1(j) = (range_wtrme1(j)-range_ideal(j) )*cosd(beta); 
         del_R_all2(j) = (range_wtrme2(j)-range_ideal_2(j) )*cosd(beta); 
        % Delay in fast time samples reference with respect to center range for
        % proper placement of LFM in RawData Matrix
         range_ref(j) = sqrt(((z_points_2(j)-z_points(j))^2)+((x_points_2(j)-x_points(j))^2)+((y_points_2(j)-y_points(j))^2));% [x,y,z]
         % td_ref(j)  = (2*range_ref(j))/radar.c-(((platform.Rc_ext+platform.Rc_2_ext))/radar.c);
         % td_ref_samples = round(round(td_ref(j)*(1/delt))+SwathSamples/2);


         % 1. Calculate the bistatic delay index
         td(j) = (range_wtrme1(j) + range_wtrme2(j))/radar.c - ((platform.Rc_ext + platform.Rc_2_ext)/radar.c);
         td_samples = round(td(j)*(1/delt) + SwathSamples/2);
         
         % 2. Define indices
         start_idx = td_samples;
         end_idx   = td_samples + PulseWidthSamples - 1;
         
         % 3. GUARD: Check bounds before writing to s_rx
         if start_idx >= 1 && end_idx <= rbins
             s_rx(j, start_idx:end_idx) = target.rcs_lin(tt) .* radar.w_r .* radar.w_a .* ...
                 TxChirp .* exp(-1j*(2*pi*radar.fc*radar.ic*(range_wtrme1(j) + range_wtrme2(j))));
         else
             % Pulse is outside the sampling window; skipping to prevent crash
         end


         %td(j) = (range_wtrme1(j)+range_wtrme2(j))/radar.c-(((platform.Rc_ext+platform.Rc_2_ext))/radar.c);
         %td_samples = round(round(td(j)*(1/delt))+SwathSamples/2);


        % Final received signal for real trajectory for UAV 2
     %   s_rx_ref(j,td_ref_samples:td_ref_samples+PulseWidthSamples-1)= radar.w_r.*radar.w_a.*TxChirp.*exp(-1j*(2*pi*radar.fc*radar.ic*(2*range_ref(j))));
        %s_rx(j,td_samples:td_samples+PulseWidthSamples-1)= target.rcs_lin(tt).*radar.w_r.*radar.w_a.*TxChirp.*exp(-1j*(2*pi*radar.fc*radar.ic*(range_wtrme1(j)+range_wtrme2(j))));
    
    end
    RawData = RawData+s_rx;
     disp(tt)
end


% Add noise
s_rx_noise = RawData;%awgn(s_rx,SNR);

% course motion compensation
    if coarse_mocomp_flag == 1
        for n = 1:(length(t_slow))
        s_rx_noise(n,:) = s_rx_noise(n,:).*exp(1j*4*pi*radar.fc*radar.ic*(0.5*(del_R_all1(n)+del_R_all2(n))));
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIGNAL PROCESSING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Range Reference Signal
s_ref = TxChirp.*exp(-1j*(2*pi*radar.fc*radar.ic*(2*range_ref(1)))) ;%TxChirp; % This is the reference signal which is direct signal obtained from UAV1 to UAV2

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
%% Range offset
for nn = 1:(length(t_slow))
r_off(nn,:)=s_mf_time(nn,:)*exp(1j*4*pi*radar.fc*radar.ic*(range_ref(1)));
end
%% Kindly mention the reference source for this algorithm
deltaR=(1*radar.lambda^2*(platform.Rc).*((Ka)*(t_slow)).^2)/(8*platform.vc^2);
range_res = (radar.c*delt)/2;
cells=round(deltaR/range_res);


 for k=1:length(t_slow)

        rcmc(k,:)=circshift(r_off(k,:),[0, -cells(k)]);

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
%%
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
% Cross range axis (m)
azi_axis = linspace(-size(sar_time,1)/2,size(sar_time,1)/2,size(sar_time,1)).*(platform.Delta_X_ext);
% Range axis (m)
ground_range_axis = linspace(-size(sar_time,2)/2,size(sar_time,2)/2,size(sar_time,2)).*(radar.c/(2*radar.Fs));

% figure;imagesc(ground_range_axis,azi_axis,abs(sar_time));title('Final Image Output ');


%% Final Plotting (Kindly check this portion ofthe code )

sar_2d =20*log10((1/platform.AzimuthSamples)*(1/SwathSamples)*(abs(sar_time)));
cmax = max(max(sar_2d)); 
range_axis = t_fast.*radar.c.*0.5;
range_axis = range_axis(1:SwathSamples);
% figure();
% imagesc(range_axis,azi_axis,sar_2d,[cmax-30 cmax]); colorbar;
% xlabel('Range (m)'), ylabel('Azimuth (m)')
% set(gca, 'YDir', 'normal');
% colormap(jet(256));
 % % Find peak corresponding to target
 % iterationData = 5; % Total number of peaks
 % for k = 1: iterationData
 %     [cross_range_index range_index] = find(sar_2d==cmax);% this command will change
 %     cross_range = azi_axis(cross_range_index);
 %     range = range_axis(range_index);
 %     cr_tgt(:,k) =   cross_range;
 %     r_tgt(:,k) =   range;
 % end
%  % Find peak corresponding to target
  [cross_range_index range_index] = find(sar_2d==cmax);
% %  Cross range estimate of target
 cross_range = azi_axis(cross_range_index);
%  % Range estimate of target
%  %range = ground_range_axis(range_index);
  range = range_axis(range_index);
% fprintf('Ground truth range of target %d m \n',xt);
% fprintf('Ground truth range of target %d m \n',(0.5*(range_ideal(600)+range_ideal_2(600))));
% fprintf('Ground truth cross range of target %d m \n',target.yt(1));
% fprintf('Measured range of target %d m \n',range);
% fprintf('Measured cross range of target %d m \n',cross_range);
end
