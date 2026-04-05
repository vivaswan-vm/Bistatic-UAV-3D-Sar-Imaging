%function [range1, cross_range1, h_est] = Exp2_InSAR(radar, platform, target)
function [r_est, x_est, y_est, z_est] = Exp2_InSAR(radar, platform, target, isac_waveform, range1, cross_range1, waveform, motion, std_dev)
Baseline_Z = 10;   % meters (vertical InSAR baseline)

platform_pos_1 = [platform.xc, platform.yc, platform.zc];

%% ==============================================================
% PASS 2 : Monostatic SAR 
%% ==============================================================

platform_2 = platform;                 
platform_2.zc = platform.zc + Baseline_Z;

[x_points_e ,y_points_e ,z_points_e,range2, cross_range2] = SAR_process_mono(platform_2, radar, target, isac_waveform, waveform, motion, std_dev);

platform_pos_2 = [platform_2.xc, platform_2.yc, platform_2.zc];

%% ==============================================================
% InSAR Height Estimation
%% ==============================================================
fprintf('\n===== InSAR HEIGHT ESTIMATION =====\n');

h_est = Insar_Core( ...
    range1, range2, ...
    radar, ...
    platform_pos_1, platform_pos_2, ...
    target.zt(1) );

% --- 3D Localization using Pass 1 (Master) ---

% Range from Pass 1
R_master = range1; 

% Cross-Range from Pass 1 (should be identical to Pass 2)
Y_master = cross_range1;

% Height estimated by InSAR
Z_est = h_est;

% Platform Position from Pass 1
Xc = platform.xc; 
Yc = platform.yc; 
Zc = platform.zc;

% Solve for X (Ground Range) using Pythagoras
% We subtract the Y and Z offsets from the total Range hypotenuse.
dist_sq_X = R_master^2 - (Y_master - Yc)^2 - (Z_est - Zc)^2;

% Check for negative value inside sqrt
if dist_sq_X < 0
    warning('Imaginary distance detected. Clamping to 0.');
    dist_sq_X = 0;
end

% Calculate Final X Coordinate
% adding sqrt because target is in +X direction relative to platform)
X_est = Xc + sqrt(dist_sq_X);

% Assign final outputs
x_est = X_est;           % our calculated X
y_est = cross_range1;    % our calculated Y
z_est = h_est;           % our calculated Z
r_est = range1; % This is the master Slant Range

%fprintf('3D Coordinates: [X=%.2f, Y=%.2f, Z=%.2f]\n', X_est, Y_master, Z_est);
