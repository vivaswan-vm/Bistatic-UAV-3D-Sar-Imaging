function [struc_constants,struc_trajectory, radar, platform, target] = initialisation_func_v2(l)

    % Propagation speed (m/s)
    struc_constants.c = physconst('LightSpeed'); 
    % Center frequency (Hz)
    struc_constants.fc = 35e9;      
    % Wavelength (m)
    struc_constants.lambda = struc_constants.c/struc_constants.fc;        
    % Antenna element spacing (m)
    struc_constants.d = struc_constants.lambda/2; 
    % inverse of wavelength
    struc_constants.lambda_inv = 1/struc_constants.lambda;
    % Sampling frequency (Hz)
    struc_constants.fs = 2.64e9;
    % Sampling time (sec)
    struc_constants.ts = 1/struc_constants.fs;
    struc_constants.PTM_len = 1;
    % Relative velocity between UAVs
    struc_constants.uav_relative_velocity = 0;
  
    struc_constants.K = 1.38e-23;
    % Noise figure (linear)
    struc_constants.F = 1.76;
    % System noise temperature
    struc_constants.Ts = 290*struc_constants.F;
    
    % Channel parameter
    % (dBm) Transmitted power
    struc_constants.Ptx = 40; %30
    % Converting transmitted power from log scale to linear scale
    struc_constants.Ptx=(10^(struc_constants.Ptx/10))*1e-3;
    
     
       %% ================= RADAR =================
radar = struct();

radar.c  = 3e8;%struc_constants.c ;
radar.ic = 1 / radar.c;

radar.fc        =  struc_constants.fc;
radar.range_res = 2;
radar.del_cr    = 5;
radar.PRF       = 1500;
radar.Tp        = 2e-6;
radar.SNR       = 10;

% Look angles 
radar.Phi_mono = 5 + (20 - 5) * rand;% max = 20 and min = 5    % UAV 1
radar.Phi_bi   = 35 + (50 - 35) * rand;% max = 50 and min = 35     % UAV 2

%% ============== DERIVED RADAR ==============
radar.lambda = radar.c / radar.fc;
radar.Bw     = radar.c / (2 * radar.range_res);
radar.Fs     = 3 * radar.Bw;
radar.PRI    = 1 / radar.PRF;

%% ================= PLATFORM =================
platform = struct();
% Squint angle (degrees)
platform.theta_sq = 0;
% Slant ranges
platform.Rc   = 800 + (1200 - 800) * rand;% max = 1.2e3 and min = 800m
platform.Rc_2 = platform.Rc;

% Velocities
platform.vc = 50;
platform.vx = 0;
platform.vz = 0;

% Geometry (KEEP EXACT)
platform.zc   = platform.Rc   * cosd(radar.Phi_mono);
platform.zc_2 = platform.Rc_2 * cosd(radar.Phi_bi);

platform.xc   = -platform.Rc   * sind(radar.Phi_mono);
platform.xc_2 = -platform.Rc_2 * sind(radar.Phi_bi);

platform.yc   = 0;
platform.yc_2 = 0;

% Synthetic aperture lengths
platform.SAL_mono = radar.lambda * platform.Rc / radar.del_cr;
platform.SAL_bi   = radar.lambda * ...
                     (0.5*(platform.Rc + platform.Rc_2)) / radar.del_cr;

platform.SAT_mono = platform.SAL_mono / platform.vc;
platform.SAT_bi   = platform.SAL_bi   / platform.vc;

platform.Delta_X  = radar.PRI * platform.vc;

%% ================= TARGET =================
target = struct();

% target.n   = 1;
 % Swath Range
 target.R0  = 50;
 % Swath Cross range
 target.CR0 = 50;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generating digital elevation model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create terrain model
xLimits         = [1 50];%[5500 6100]; % x-axis limits of terrain (m)
yLimits         = [1 50]; % y-axis limits of terrain (m)
roughnessFactor = 1.75;       % Roughness factor
initialHgt      = 0;          % Initial height (m)
initialPerturb  = 20;        % Overall height of map (m) 
numIter         = 5;          % Number of iterations
[x_terrain,y_terrain,z_terrain] = helperRandomTerrainGenerator(roughnessFactor,initialHgt, ....
    initialPerturb,xLimits(1),xLimits(2), ...
    yLimits(1),yLimits(2),numIter);
z_terrain(z_terrain < 0) = 0; % Fill-in areas below 0
% converting matrix to array of x,y and z coordinates
x_dem = (x_terrain(:)); 
x_dem = x_dem+rand(length(x_dem),1);
y_dem = (y_terrain(:));
y_dem = y_dem+rand(length(x_dem),1);
z_dem = (z_terrain(:));
% RCS of each DEM point
rcs_lin_dem = abs(randn(size(z_terrain)));
rcs_lin_dem(z_terrain>10) = 2;
rcs_lin_dem = rcs_lin_dem(:);
x = x_dem.';
% Plot simulated terrain
%helperPlotSimulatedTerrain(x,y_dem,z_terrain)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TARGET PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loading car data
%x-coordinates of car
x_car = load(project_data_path("car_x.mat"));
x_points = x_car.x_centre;
%y-coordinates of car
y_car = load(project_data_path("car_y.mat"));
y_points = y_car.y_centre;
%z-coordinates of car
z_car = load(project_data_path("car_z.mat"));
z_points = z_car.z_centre;

N = length(z_points);
K = 1801; % Numberr of body parts that are not shadowed

indices = randperm(N, K);   % Bernoulli Random unique indices
%Range of car point scatterers (m)
xt_o = [x_points(indices)];%[x_points(1:5:5499) x_points(5500:2:6900)];% main body and wheeels
% Adjust range according to the terrain model
mean_xt = 20;
xt_rand =  mean_xt*rand(1,1);
xt_car = (xt_o+xt_rand).';
%Cross range of car point scatterers (m)
yt_o = [y_points(indices)];%[y_points(1:5:5499)  y_points(5500:2:6900)];% main body and wheeels
% Adjust cross range according to the terrain model
mean_yt = 20;
yt_rand =  mean_yt*rand(1,1);
yt_car = (yt_o+yt_rand).';
% Height of of car point scatterers (m)
zt_o = [z_points(indices)];%[z_points(1:5:5499) z_points(5500:2:6900)];% main body and wheeels
mean_zt = 10;
zt_rand =  mean_zt*rand(1,1);
zt_car =(zt_o+zt_rand).';
% RCS of each car point scatterer
% mean_rcs = 10;
% rcs_lin_car = (mean_rcs .*rand(size(zt_car)));
rcs_c = load(project_data_path("car_rcs.mat"));
rcs_lin_car  = rcs_c.r1(:,l);
%% Target Locations
%%%Only automotive target
 % xt = (xt_car);
 % yt = (yt_car);
 % zt = (zt_car);
 % rcs_lin = (rcs_lin_car); 

%Integrate DEM and automotive target
 target.xt = vertcat(x_dem(1:3:900),xt_car);
 target.yt = vertcat(y_dem(1:3:900),yt_car);
 target.zt = vertcat(z_dem(1:3:900),(zt_car));
 target.rcs_lin = vertcat(rcs_lin_dem(1:3:900),rcs_lin_car); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% target.xt = [5; -5; 0; -5; 5];
% target.yt = [5; -5; 0; 5; -5];
% target.zt = [0; 0; 0; 0; 0];
% target.rcs_lin = [10; 10; 10; 10; 10];
%% ============== AZIMUTH SWATH ==============
platform.AzimuthSwathTime = (2 * target.CR0) / platform.vc;
platform.AzimuthSamples  = round(platform.AzimuthSwathTime * radar.PRF);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

   struc_constants.channelType = 'friis';
       % UAV1 initial position at t=0sec
       struc_trajectory.radar1_pos = [platform.xc ,platform.yc ,platform.zc ]; 
       % UAV2 initial position at t=0sec
       struc_trajectory.radar2_pos = [platform.xc_2 ,platform.yc_2 ,platform.zc_2]; 
       % initial range of UAV1 to UAV2
       struc_trajectory.range_tgt = norm(struc_trajectory.radar2_pos-struc_trajectory.radar1_pos);
       % cos of angle between UAV1 and UAV2
       struc_trajectory.cos_thetat = acosd(struc_trajectory.radar2_pos(1,2)/struc_trajectory.range_tgt);
       % target velocity vector
       struc_trajectory.relative_velocity_vector = [-struc_constants.uav_relative_velocity*sind(struc_trajectory.cos_thetat),struc_constants.uav_relative_velocity*cosd(struc_trajectory.cos_thetat),0]; 
 

end

