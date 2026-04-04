
function [r_est, x_est, y_est, z_est] = Exp1_Bistatic_SAR(struc_constants,struc_trajectory,radar, platform, target, waveform, motion, std_dev)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PART 2
% Determination of Range and Cross Range in monostatic and bistatic

% Monostatic SAR modeling
[x_points_e ,y_points_e ,z_points_e,range_mono,cross_range_mono] = SAR_process_mono(platform,radar,target, waveform, motion, std_dev);
% BIstatic SAR modeling
[range_bi,cross_range_bi] = SAR_process_bi(x_points_e ,y_points_e ,z_points_e,struc_constants,struc_trajectory,radar, platform, target, waveform, motion, std_dev);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Loop for all target range and cross range one by one
%% PART 2
% Determination of theta_T and theta_R
Rt = range_mono;
Rr = (2*range_bi)-Rt;
% co-ordinates of UAV 1
point1 = [platform.xc, platform.yc, platform.zc]; 
point2 = [platform.xc_2, platform.yc_2, platform.zc_2]; 
% Calculate the Euclidean distance
L = sqrt(sum((point2 - point1).^2));
% Calculate sin(theta_R) and sin(theta_T)
sin_theta_R = ((Rt + Rr)^2 - L^2 - 2*Rr*(Rt + Rr)) / (2*Rr*L);
sin_theta_T = (2*Rt*(Rt + Rr) - (Rt + Rr)^2 + L^2) / (2*Rt*L);

% Compute angles in radians
theta_R = asin(sin_theta_R);
theta_T = asin(sin_theta_T);

beta = theta_T-theta_R;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
%% Determining of 3D coordinates of the target
% INPUTS

beta = rad2deg(beta);
A = point2;    % Coordinates of A (UAV 1)
B = point1;    % Coordinates of B (UAV 1)
 
a = Rt;            % |AC|
b = Rr;            % |BC| 

%STEP 1:



%%%%%%%%%%%%%%%%%%%%%%
% length of sides
  sideAB = sqrt(sum((B - A).^2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xA = A(1); yA = A(2); zA = A(3);
xB = B(1); yB = B(2); zB = B(3);


D = b^2-a^2+(xB^2+yB^2+zB^2)-(xA^2+yA^2+zA^2);
Px = 2*(xB-xA);
Py = 2*(yB-yA);
Pz = 2*(zB-zA);

yC = cross_range_mono;%C(2);
% Solving for xC and yC
syms xC 

zC = (D-(Px*xC)-(Py*yC))/Pz;
eq1 = (xA-xC)*(xB-xC)+(yA-yC)*(yB-yC)+(zA-zC)*(zB-zC) == (b*a*cosd(beta));

sol = solve([eq1], [xC]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xC1 = sol(1);
xC1 = double(xC1);
zC1 = (D-(Px*xC1)-(Py*yC))/Pz;

xC2 = sol(2);
xC2 = double(xC2);
zC2 = (D-(Px*xC2)-(Py*yC))/Pz;


C1 = [xC1 ,yC, zC1 ];
C2 = [xC2 ,yC, zC2 ];

%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Maximum platform height
z_max = max(platform.zc, platform.zc_2);

% Check validity
valid_C1 = (zC1 <= z_max);
valid_C2 = (zC2 <= z_max);

if valid_C1 && ~valid_C2
    x_est = xC1;
    z_est = zC1;

elseif valid_C2 && ~valid_C1
    x_est = xC2;
    z_est = zC2;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%x_est = xC2;            % The X coordinate found by the solver
y_est = cross_range_mono; 
%z_est = zC2;            % The Z coordinate (h_est)
r_est = range_mono;


end
