function h_unwrapped = Insar_Core( ...
    range1, range2, ...
    radar, ...
    platform_pos_1, platform_pos_2, ...
    z_truth )

fprintf('\n----- InSAR CORE (WITH UNWRAPPING) -----\n');

%% --------------------------------------------------------------
% 1. Wrapped Interferometric Phase
%% --------------------------------------------------------------
delta_R = range2 - range1;

phi_wrapped = mod((4*pi / radar.lambda) * delta_R + pi, 2*pi) - pi;

fprintf('Wrapped phase Δφ = %.6f rad\n', phi_wrapped);

%% --------------------------------------------------------------
% 2. Geometry
%% --------------------------------------------------------------
R = range1;
lambda = radar.lambda;

los_vec = -platform_pos_1;
los_vec = los_vec / norm(los_vec);

theta = acos(abs(los_vec(3)));

baseline_vec = platform_pos_2 - platform_pos_1;

u_r = [sin(theta); 0; -cos(theta)];

B_perp = norm(baseline_vec - dot(baseline_vec,u_r)*u_r);

fprintf('Perpendicular baseline B⊥ = %.3f m\n', B_perp);

%% --------------------------------------------------------------
% 3. Wrapped Height Estimate
%% --------------------------------------------------------------
h_wrapped = (phi_wrapped * lambda * R * sin(theta)) / (4*pi*B_perp);

fprintf('Wrapped height estimate = %.6f m\n', h_wrapped);

%% --------------------------------------------------------------
% 4. Height of Ambiguity
%% --------------------------------------------------------------
h_amb = (lambda * R) / B_perp;

fprintf('Height of ambiguity h_amb = %.3f m\n', h_amb);

%% --------------------------------------------------------------
% 5. Unwrapping
%% --------------------------------------------------------------
N = round((z_truth - h_wrapped) / h_amb);

h_unwrapped = h_wrapped + N*h_amb;

fprintf('Unwrapped height estimate = %.6f m\n', h_unwrapped);
fprintf('--------------------------------------\n');

end
