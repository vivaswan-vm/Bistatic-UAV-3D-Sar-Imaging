

function [land_clutter] = generate_landclutter(avg_vel,radar,ground_range_axis,azi_axis)

sigma = 0.5;
sigma_0_dB = 10*log10(sigma);

% Radar Parameters
%Operating frequecy(Hz)
fc_Hz = radar.fc;
% Speed of light (m/s)
c_mps = radar.c;
% wavelength(m)
lambda_m=radar.lambda;
% Bandwidth (GHz)
BW_Hz = radar.Bw;
% Pulse width (seconds)
tau_sec = 1/BW_Hz;
% Grazing angle (degrees)
psi_deg = 30;
% beamwidth (radians)
bw_rad = (pi/180)*78;
%Transmitted power 
P_W = 40;
%Antenna gain 
G_ant=1;
%multiplication term to transmitted power and received power so minimum
%received power become -80dBm
A=320;
P_W=A*P_W;
% clutter parameter 
% Average stable normalized backscatter (dB)
sigma_0_st_dB = sigma_0_dB;
% sigma_0_st_dB =10^(sigma_0_dB/10);
% Fluctuating normalized backscatter (dB)
asq = 10^4;
% Average normalized backscatter (dB)
sigma_0 = 10^(sigma_0_st_dB/10)*(1+1/asq);
%Average wind velocity
u_mps=avg_vel;
% Doppler spread due to windspeed (Hz)
delf = 1.23*(3.2/(lambda_m*100))*(u_mps^1.3);
% Power factor (n)
npow=2*((u_mps+2)/(u_mps+2))*((100/(fc_Hz/10^9))^0.2);
%Spectral density G_0 for zero frequecy
G_0=1;
% Range axis (Maximum Range 50m)
r_m = ground_range_axis;
% number of range cells
nr = length(r_m);
% Doppler axis
fd = azi_axis;
% number of Doppler cells
nFd = length(fd);
% range-Doppler ambiguity map
Xhi_rfd = zeros(nr,nFd);
% Range-Doppler ambiguity map
 for n = 1:nr
    % Single instance of sigma_0
    sigma_0_i = exprnd(sigma_0);
    % Clutter power (Watts)
%     clutter = P_W*sigma_0_i*bw_rad*c_mps*tau_sec*0.5*(1/(cosd(psi_deg)))/((4*pi)^2*(r_m(n)^3));
    sigma_c= r_m(n)*sigma_0_i*bw_rad*c_mps*tau_sec*0.5*(1/cosd(psi_deg));
   
    %Factor due to radar range equation
    range_eq_factor=(P_W*(G_ant^2)*(lambda_m^2))/(((4*pi)^3)*(r_m(n)^4));
    clutter_power=sigma_c*range_eq_factor;
    for m = 1:nFd
        G = G_0*(1/(1+(fd(m)/delf)^npow));
%         G_dash=clutter_power*asq+G;
%         clutter_power_total=G_dash;
        clutter_power_total=clutter_power*G;
        clutter_volts=sqrt(clutter_power_total/2);
        Xhi_rfd(n,m)=(clutter_volts*randn(1,1))+(1i*clutter_volts*randn(1,1));
    end
 end 
 
land_clutter= 1e6.*Xhi_rfd;
%% Plots
  maxx = max(max(10*log10(abs(land_clutter))));
figure()
imagesc(fd,r_m,10*log10(10^3*abs(land_clutter)),[maxx-60,maxx]);
colorbar
ax = gca;
ax.YDir = 'normal';
ylabel('range (m)')
xlabel('Doppler(hz)')
colormap(jet(256));
end


