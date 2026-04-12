clc;
clear;

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'src')));


%% ==============================
% Flags
% ==============================

Method_Flag       = [1 2];   % 1 = Bistatic, 2 = InSAR
Waveform_Flag     = [1 2];   % 1 = FMCW, 2 = ISAC
Motion_Comp_Flag  = [1 2];   % 1 = With, 2 = Without

Number_of_Iterations = 200;
Std_Dev_Vel = [0.1  0.5  1.5 1 5 7 10]; % Log Spaced Sweep

%Test Setup
% 
% Method_Flag       = [2];   
% Waveform_Flag     = [1];  
% Motion_Comp_Flag  = [1];  
% 
% Number_of_Iterations = 1;
% Std_Dev_Vel = [0.1]; % Log Spaced Sweep


% Results Tensor
% ==============================

% Dim 1 → Method
% Dim 2 → Waveform
% Dim 3 → Motion Comp
% Dim 4 → Std Dev Vel
% Dim 5 → Metric (x,y,z,3D)

Results = zeros(length(Method_Flag), ...
                length(Waveform_Flag), ...
                length(Motion_Comp_Flag), ...
                length(Std_Dev_Vel), ...
                4);

err_x = zeros(length(Method_Flag), ...
              length(Waveform_Flag), ...
              length(Motion_Comp_Flag), ...
              length(Std_Dev_Vel));
err_y = zeros(length(Method_Flag), ...
              length(Waveform_Flag), ...
              length(Motion_Comp_Flag), ...
              length(Std_Dev_Vel));
err_z = zeros(length(Method_Flag), ...
              length(Waveform_Flag), ...
              length(Motion_Comp_Flag), ...
              length(Std_Dev_Vel));
err_3D = zeros(length(Method_Flag), ...
               length(Waveform_Flag), ...
               length(Motion_Comp_Flag), ...
               length(Std_Dev_Vel));

%% ==============================
% Main Experiment Loops
% ==============================

rng(11);% fixing seed value for all scenarios
static_config = initialisation_static_config();
for l = 1:Number_of_Iterations
    % TIMING ANALYSIS: comment out these two lines when not needed
    iterationTimer = tic;

    % Initialization
    [struc_constants,struc_trajectory, radar, platform, target] = ...
        initialisation_monte_carlo_scenario(static_config, l);

    % Ground truth
    peak_rcs = max(target.rcs_lin);
    row_rcs  = find(target.rcs_lin == peak_rcs,1);

    x_gt = target.xt(row_rcs);
    y_gt = target.yt(row_rcs);
    z_gt = target.zt(row_rcs);

    for waveform = 1:length(Waveform_Flag)
    for motion = 1:length(Motion_Comp_Flag)
    for k = 1:length(Std_Dev_Vel)

        [x_points_e, y_points_e, z_points_e, range_mono, cross_range_mono] = ...
            SAR_process_mono(platform, radar, target, static_config.isac_waveform, ...
                             waveform, motion, Std_Dev_Vel(k));

        for method = 1:length(Method_Flag)

        %% ==============================
        % Select Method
        %% ==============================

        if Method_Flag(method) == 1
            % Bistatic
        [~, X_est, Y_est, Z_est] = ...
            Exp1_Bistatic_SAR(struc_constants, ...
                              struc_trajectory, ...
                              radar, ...
                              platform, ...
                              target, ...
                              static_config.isac_waveform, ...
                              x_points_e, ...
                              y_points_e, ...
                              z_points_e, ...
                              range_mono, ...
                              cross_range_mono, ...
                              waveform, ...
                              motion, ...
                              Std_Dev_Vel(k));
        else
            % InSAR
        [~, X_est, Y_est, Z_est] = ...
            Exp2_InSAR(radar, ...
                       platform, ...
                       target, ...
                       static_config.isac_waveform, ...
                       range_mono, ...
                       cross_range_mono, ...
                       waveform, ...
                       motion, ...
                       Std_Dev_Vel(k));
        end

        %% ==============================
        % Accumulate Properly
        %% ==============================

        dx = X_est - x_gt;
        dy = Y_est - y_gt;
        dz = Z_est - z_gt;

        err_x(method,waveform,motion,k)  = err_x(method,waveform,motion,k)  + dx^2;
        err_y(method,waveform,motion,k)  = err_y(method,waveform,motion,k)  + dy^2;
        err_z(method,waveform,motion,k)  = err_z(method,waveform,motion,k)  + dz^2;
        err_3D(method,waveform,motion,k) = err_3D(method,waveform,motion,k) + dx^2 + dy^2 + dz^2;

        end
    end
    end
    end

    % TIMING ANALYSIS: comment out this line when not needed
    fprintf('Monte Carlo iteration %d completed in %.3f seconds\n', l, toc(iterationTimer));
end

%% ==============================
% Compute RMSE
%% ==============================

N = Number_of_Iterations;
Results(:,:,:,:,1) = sqrt(err_x  / N);
Results(:,:,:,:,2) = sqrt(err_y  / N);
Results(:,:,:,:,3) = sqrt(err_z  / N);
Results(:,:,:,:,4) = sqrt(err_3D / N);

%% ==============================
% Convert Results Tensor to Table
%% ==============================

Method_Names   = ["Bistatic","InSAR"];
Waveform_Names = ["FMCW","ISAC"];
Motion_Names   = ["WithMC","WithoutMC"];

selectedMethodNames = Method_Names(Method_Flag);
selectedWaveformNames = Waveform_Names(Waveform_Flag);
selectedMotionNames = Motion_Names(Motion_Comp_Flag);

row = 1;

for method = 1:length(Method_Flag)
for waveform = 1:length(Waveform_Flag)
for motion = 1:length(Motion_Comp_Flag)
for k = 1:length(Std_Dev_Vel)

    Method(row,1)   = selectedMethodNames(method);
    Waveform(row,1) = selectedWaveformNames(waveform);
    Motion(row,1)   = selectedMotionNames(motion);
    StdVel(row,1)   = Std_Dev_Vel(k);

    RMSE_x(row,1)   = Results(method,waveform,motion,k,1);
    RMSE_y(row,1)   = Results(method,waveform,motion,k,2);
    RMSE_z(row,1)   = Results(method,waveform,motion,k,3);
    RMSE_3D(row,1)  = Results(method,waveform,motion,k,4);

    row = row + 1;

end
end
end
end

Results_Table = table(Method,Waveform,Motion,StdVel,...
                      RMSE_x,RMSE_y,RMSE_z,RMSE_3D);

disp(Results_Table);


%% ==============================
% Plot RMSE_3D vs Velocity Std Dev
%% ==============================

figure;
sgtitle('RMSE 3D vs Velocity Std Dev');

for method = 1:length(Method_Flag)
    
    subplot(1,2,method);
    hold on;
    grid on;
    
    legend_entries = {};
    curve_idx = 1;
    
    for waveform = 1:length(Waveform_Flag)
        for motion = 1:length(Motion_Comp_Flag)
            
            rmse_curve = squeeze(Results(method,waveform,motion,:,4));
            
            plot(Std_Dev_Vel, rmse_curve, '-o', 'LineWidth', 1.5);
            
            legend_entries{curve_idx} = ...
                selectedWaveformNames(waveform) + " - " + selectedMotionNames(motion);
            
            curve_idx = curve_idx + 1;
        end
    end
    
    set(gca,'XScale','log');  
    
    xlabel('Velocity Std Dev');
    ylabel('RMSE 3D');
    title(selectedMethodNames(method));
    legend(legend_entries,'Location','northwest');
    
end
