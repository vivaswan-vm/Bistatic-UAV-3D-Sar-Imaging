function static_config = initialisation_static_config()

    static_config = struct();

    % Propagation and system constants
    static_config.static_constants = struct();
    static_config.static_constants.c = physconst('LightSpeed');
    static_config.static_constants.fc = 35e9;
    static_config.static_constants.lambda = ...
        static_config.static_constants.c / static_config.static_constants.fc;
    static_config.static_constants.d = static_config.static_constants.lambda / 2;
    static_config.static_constants.lambda_inv = 1 / static_config.static_constants.lambda;
    static_config.static_constants.fs = 2.64e9;
    static_config.static_constants.ts = 1 / static_config.static_constants.fs;
    static_config.static_constants.PTM_len = 1;
    static_config.static_constants.uav_relative_velocity = 0;
    static_config.static_constants.K = 1.38e-23;
    static_config.static_constants.F = 1.76;
    static_config.static_constants.Ts = 290 * static_config.static_constants.F;
    static_config.static_constants.Ptx = (10^(40 / 10)) * 1e-3;
    static_config.static_constants.channelType = 'friis';

    % Radar parameters that do not change per iteration
    static_config.static_radar = struct();
    static_config.static_radar.c = 3e8;
    static_config.static_radar.ic = 1 / static_config.static_radar.c;
    static_config.static_radar.fc = static_config.static_constants.fc;
    static_config.static_radar.range_res = 2;
    static_config.static_radar.del_cr = 5;
    static_config.static_radar.PRF = 1500;
    static_config.static_radar.Tp = 2e-6;
    static_config.static_radar.SNR = 10;
    static_config.static_radar.lambda = static_config.static_radar.c / static_config.static_radar.fc;
    static_config.static_radar.Bw = static_config.static_radar.c / (2 * static_config.static_radar.range_res);
    static_config.static_radar.Fs = 3 * static_config.static_radar.Bw;
    static_config.static_radar.PRI = 1 / static_config.static_radar.PRF;

    % Platform settings
    static_config.static_platform = struct();
    static_config.static_platform.theta_sq = 0;
    static_config.static_platform.vc = 50;
    static_config.static_platform.vx = 0;
    static_config.static_platform.vz = 0;

    % Target and terrain settings
    static_config.static_target = struct();
    static_config.static_target.R0 = 50;
    static_config.static_target.CR0 = 50;

    static_config.static_terrain = struct();
    static_config.static_terrain.xLimits = [1 50];
    static_config.static_terrain.yLimits = [1 50];
    static_config.static_terrain.roughnessFactor = 1.75;
    static_config.static_terrain.initialHgt = 0;
    static_config.static_terrain.initialPerturb = 20;
    static_config.static_terrain.numIter = 5;

    static_config.monte_carlo_randomization = struct();
    static_config.monte_carlo_randomization.phi_mono_range = [5 20];
    static_config.monte_carlo_randomization.phi_bi_range = [35 50];
    static_config.monte_carlo_randomization.Rc_range = [800 1200];
    static_config.monte_carlo_randomization.visible_scatterers = 1801;
    static_config.monte_carlo_randomization.mean_xt = 20;
    static_config.monte_carlo_randomization.mean_yt = 20;
    static_config.monte_carlo_randomization.mean_zt = 10;

    % Static data assets
    static_config.static_assets = struct();
    x_car = load(project_data_path("car_x.mat"));
    y_car = load(project_data_path("car_y.mat"));
    z_car = load(project_data_path("car_z.mat"));
    rcs_c = load(project_data_path("car_rcs.mat"));
    isac_waveform = load(project_data_path("ISAC_waveform_v2.mat"));

    static_config.static_assets.x_points = x_car.x_centre;
    static_config.static_assets.y_points = y_car.y_centre;
    static_config.static_assets.z_points = z_car.z_centre;
    static_config.static_assets.rcs_table = rcs_c.r1;
    static_config.isac_waveform = isac_waveform.tx;
end
