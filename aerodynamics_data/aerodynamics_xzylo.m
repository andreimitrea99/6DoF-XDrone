%% --- Parameters
limit_angle_rad = deg2rad(90);
alpha_range = linspace(-limit_angle_rad, limit_angle_rad, 200);
% x_AC = 0.25; % aerodynamic center (fraction of chord)

% File paths
path_base = pwd;  % returns the current working directory
file_CL  = fullfile(path_base, 'aerodynamics_data', 'C_L.txt');
file_CD  = fullfile(path_base, 'aerodynamics_data', 'C_D.txt');
file_COP = fullfile(path_base, 'aerodynamics_data', 'COP.txt');

%% ============================================================
%  1. LIFT CURVE (C_L)
% =============================================================
Lift_graph = readmatrix(file_CL);
alpha_cl = Lift_graph(:,1);
C_L = Lift_graph(:,2);

alpha_cl_sym = [-flip(alpha_cl(2:end)); alpha_cl];
C_L_sym = [-flip(C_L(2:end)); C_L];

alpha_rad_CL = deg2rad(alpha_cl_sym);
C_L_sym_smooth = smoothdata(C_L_sym, 'loess', 8);

% CL_fun = @(alpha) interp1(alpha_rad_CL, C_L_sym_smooth, alpha, 'pchip', 'extrap');

%% ============================================================
%  2. DRAG CURVE (C_D)
% =============================================================
Drag_graph = readmatrix(file_CD);
alpha_drag = Drag_graph(:,1);
C_D = Drag_graph(:,2);

alpha_drag_sym = [-flip(alpha_drag(2:end)); alpha_drag];
C_D_sym = [flip(C_D(2:end)); C_D];

alpha_rad_CD = deg2rad(alpha_drag_sym);
C_D_sym_smooth = smoothdata(C_D_sym, 'loess', 8);

% CD_fun = @(alpha) interp1(alpha_rad_CD, C_D_sym_smooth, alpha, 'pchip', 'extrap');

%% ============================================================
%  3. CENTER OF PRESSURE (COP)
% =============================================================
COP_graph = readmatrix(file_COP);
alpha_cop = COP_graph(:,1);
COP = COP_graph(:,2);

alpha_cop_sym = [-flip(alpha_cop(2:end)); alpha_cop];
COP_sym = [flip(COP(2:end)); COP];

alpha_rad_COP = deg2rad(alpha_cop_sym);
COP_sym_smooth = smoothdata(COP_sym, 'loess', 8);

% COP_frac_fun = @(alpha) interp1(alpha_rad_COP, COP_sym_smooth, alpha, 'pchip', 'extrap')./ 100;

%% ============================================================
%  4. Pitching Moment about Aerodynamic Center
% =============================================================

% Cm_fun = @(alpha) (COP_frac_fun(alpha) - x_AC) .* (CL_fun(alpha) .* cos(alpha) + CD_fun(alpha) .* sin(alpha));

