function [monte_carlo_constants, monte_carlo_trajectory, monte_carlo_radar, ...
          monte_carlo_platform, monte_carlo_target] = ...
    initialisation_monte_carlo_scenario(static_config, mc_iter)

    monte_carlo_constants = static_config.static_constants;
    monte_carlo_radar = static_config.static_radar;
    monte_carlo_platform = static_config.static_platform;
    monte_carlo_target = static_config.static_target;

    % Iteration-specific radar geometry
    monte_carlo_radar.Phi_mono = static_config.monte_carlo_randomization.phi_mono_range(1) + ...
        diff(static_config.monte_carlo_randomization.phi_mono_range) * rand;
    monte_carlo_radar.Phi_bi = static_config.monte_carlo_randomization.phi_bi_range(1) + ...
        diff(static_config.monte_carlo_randomization.phi_bi_range) * rand;

    % Iteration-specific platform geometry
    monte_carlo_platform.Rc = static_config.monte_carlo_randomization.Rc_range(1) + ...
        diff(static_config.monte_carlo_randomization.Rc_range) * rand;
    monte_carlo_platform.Rc_2 = monte_carlo_platform.Rc;

    monte_carlo_platform.zc = monte_carlo_platform.Rc * cosd(monte_carlo_radar.Phi_mono);
    monte_carlo_platform.zc_2 = monte_carlo_platform.Rc_2 * cosd(monte_carlo_radar.Phi_bi);
    monte_carlo_platform.xc = -monte_carlo_platform.Rc * sind(monte_carlo_radar.Phi_mono);
    monte_carlo_platform.xc_2 = -monte_carlo_platform.Rc_2 * sind(monte_carlo_radar.Phi_bi);
    monte_carlo_platform.yc = 0;
    monte_carlo_platform.yc_2 = 0;
    monte_carlo_platform.SAL_mono = monte_carlo_radar.lambda * monte_carlo_platform.Rc / monte_carlo_radar.del_cr;
    monte_carlo_platform.SAL_bi = monte_carlo_radar.lambda * ...
        (0.5 * (monte_carlo_platform.Rc + monte_carlo_platform.Rc_2)) / monte_carlo_radar.del_cr;
    monte_carlo_platform.SAT_mono = monte_carlo_platform.SAL_mono / monte_carlo_platform.vc;
    monte_carlo_platform.SAT_bi = monte_carlo_platform.SAL_bi / monte_carlo_platform.vc;
    monte_carlo_platform.Delta_X = monte_carlo_radar.PRI * monte_carlo_platform.vc;

    % Terrain generation
    [x_terrain, y_terrain, z_terrain] = helperRandomTerrainGenerator( ...
        static_config.static_terrain.roughnessFactor, ...
        static_config.static_terrain.initialHgt, ...
        static_config.static_terrain.initialPerturb, ...
        static_config.static_terrain.xLimits(1), static_config.static_terrain.xLimits(2), ...
        static_config.static_terrain.yLimits(1), static_config.static_terrain.yLimits(2), ...
        static_config.static_terrain.numIter);

    z_terrain(z_terrain < 0) = 0;
    x_dem = x_terrain(:);
    x_dem = x_dem + rand(length(x_dem), 1);
    y_dem = y_terrain(:);
    y_dem = y_dem + rand(length(x_dem), 1);
    z_dem = z_terrain(:);

    rcs_lin_dem = abs(randn(size(z_terrain)));
    rcs_lin_dem(z_terrain > 10) = 2;
    rcs_lin_dem = rcs_lin_dem(:);

    % Automotive target generation
    z_points = static_config.static_assets.z_points;
    N = length(z_points);
    K = static_config.monte_carlo_randomization.visible_scatterers;
    indices = randperm(N, K);

    xt_o = static_config.static_assets.x_points(indices);
    xt_car = (xt_o + static_config.monte_carlo_randomization.mean_xt * rand(1,1)).';

    yt_o = static_config.static_assets.y_points(indices);
    yt_car = (yt_o + static_config.monte_carlo_randomization.mean_yt * rand(1,1)).';

    zt_o = static_config.static_assets.z_points(indices);
    zt_car = (zt_o + static_config.monte_carlo_randomization.mean_zt * rand(1,1)).';

    rcs_lin_car = static_config.static_assets.rcs_table(:, mc_iter);

    % Assemble target scene
    monte_carlo_target.xt = vertcat(x_dem(1:3:900), xt_car);
    monte_carlo_target.yt = vertcat(y_dem(1:3:900), yt_car);
    monte_carlo_target.zt = vertcat(z_dem(1:3:900), zt_car);
    monte_carlo_target.rcs_lin = vertcat(rcs_lin_dem(1:3:900), rcs_lin_car);

    monte_carlo_platform.AzimuthSwathTime = (2 * monte_carlo_target.CR0) / monte_carlo_platform.vc;
    monte_carlo_platform.AzimuthSamples = round(monte_carlo_platform.AzimuthSwathTime * monte_carlo_radar.PRF);

    % Trajectory setup
    monte_carlo_trajectory = struct();
    monte_carlo_trajectory.radar1_pos = [monte_carlo_platform.xc, monte_carlo_platform.yc, monte_carlo_platform.zc];
    monte_carlo_trajectory.radar2_pos = [monte_carlo_platform.xc_2, monte_carlo_platform.yc_2, monte_carlo_platform.zc_2];
    monte_carlo_trajectory.range_tgt = norm( ...
        monte_carlo_trajectory.radar2_pos - monte_carlo_trajectory.radar1_pos);
    monte_carlo_trajectory.cos_thetat = acosd( ...
        monte_carlo_trajectory.radar2_pos(1,2) / monte_carlo_trajectory.range_tgt);
    monte_carlo_trajectory.relative_velocity_vector = [ ...
        -monte_carlo_constants.uav_relative_velocity * sind(monte_carlo_trajectory.cos_thetat), ...
         monte_carlo_constants.uav_relative_velocity * cosd(monte_carlo_trajectory.cos_thetat), ...
         0];
end
