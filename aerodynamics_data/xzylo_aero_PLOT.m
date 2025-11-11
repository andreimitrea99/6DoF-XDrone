%% Aerodynamic Coefficients and Pitching Moment Calculation
clear; close all; clc;
%% Parameters
limit_angle_rad = deg2rad(90);
alpha_range = linspace(-limit_angle_rad, limit_angle_rad, 200);
x_AC = 0.23; % aerodynamic center (fraction of chord)

% File paths (adjust if needed)
path_base = 'C:\Users\mitre\Desktop\MATLAB\PhD MATLAB Projects\6DoF-Try2\aerodynamics_data\';
file_CL  = fullfile(path_base, 'C_L.txt');
file_CD  = fullfile(path_base, 'C_D.txt');
file_COP = fullfile(path_base, 'COP.txt');
%% ============================================================
%  1. LIFT CURVE (C_L)
% =============================================================
Lift_graph = readmatrix(file_CL);
alpha_cl = Lift_graph(:,1);
C_L = Lift_graph(:,2);
% Symmetric (antisymmetric lift)
alpha_cl_sym = [-flip(alpha_cl(2:end)); alpha_cl];
C_L_sym = [-flip(C_L(2:end)); C_L];
% Convert to radians
alpha_rad_CL = deg2rad(alpha_cl_sym);
% Smooth and interpolate
C_L_sym_smooth = smoothdata(C_L_sym, 'loess', 5);
C_L_fit = interp1(alpha_rad_CL, C_L_sym_smooth, alpha_range, 'pchip');
% Plot
figure;
plot(rad2deg(alpha_rad_CL), C_L_sym, 'o', 'DisplayName','Raw Data');
hold on;
plot(rad2deg(alpha_rad_CL), C_L_sym_smooth, 'g--', 'LineWidth',1.5, 'DisplayName','Smoothed');
plot(rad2deg(alpha_range), C_L_fit, 'r-', 'LineWidth',1.5, 'DisplayName','Interpolated');
xlabel('\alpha [deg]');
ylabel('C_L');
title('Lift Curve (C_L)');
legend('Location','best'); grid on;
%% ============================================================
%  2. DRAG CURVE (C_D)
% =============================================================
Drag_graph = readmatrix(file_CD);
alpha_drag = Drag_graph(:,1);
C_D = Drag_graph(:,2);
% Symmetric (even drag)
alpha_drag_sym = [-flip(alpha_drag(2:end)); alpha_drag];
C_D_sym = [flip(C_D(2:end)); C_D];
% Convert to radians
alpha_rad_CD = deg2rad(alpha_drag_sym);
% Smooth and interpolate
C_D_sym_smooth = smoothdata(C_D_sym, 'loess', 1);
C_D_fit = interp1(alpha_rad_CD, C_D_sym_smooth, alpha_range, 'pchip');
% Plot
figure;
plot(rad2deg(alpha_rad_CD), C_D_sym, 'o', 'DisplayName','Raw Data');
hold on;
plot(rad2deg(alpha_rad_CD), C_D_sym_smooth, 'g--', 'LineWidth',1.5, 'DisplayName','Smoothed');
plot(rad2deg(alpha_range), C_D_fit, 'r-', 'LineWidth',1.5, 'DisplayName','Interpolated');
xlabel('\alpha [deg]');
ylabel('C_D');
title('Drag Curve (C_D)');
legend('Location','best'); grid on;
%% ============================================================
%  3. CENTER OF PRESSURE (COP)
% =============================================================
COP_graph = readmatrix(file_COP);
alpha_cop = COP_graph(:,1);
COP = COP_graph(:,2);
% Symmetric (even function)
alpha_cop_sym = [-flip(alpha_cop(2:end)); alpha_cop];
COP_sym = [flip(COP(2:end)); COP];
% Convert to radians
alpha_rad_COP = deg2rad(alpha_cop_sym);
% Smooth and interpolate
COP_sym_smooth = smoothdata(COP_sym, 'loess', 6);
COP_fit = interp1(alpha_rad_COP, COP_sym_smooth, alpha_range, 'pchip');
% Plot
figure;
plot(rad2deg(alpha_rad_COP), COP_sym, 'o', 'DisplayName','Raw Data');
hold on;
plot(rad2deg(alpha_rad_COP), COP_sym_smooth, 'g--', 'LineWidth',1.5, 'DisplayName','Smoothed');
plot(rad2deg(alpha_range), COP_fit, 'r-', 'LineWidth',1.5, 'DisplayName','Interpolated');
xlabel('\alpha [deg]');
ylabel('Center of Pressure (% chord)');
title('Center of Pressure (COP)');
legend('Location','best'); grid on;
%% ============================================================
%  4. PITCHING MOMENT ABOUT AERODYNAMIC CENTER (x_AC = 0.25c)
% =============================================================
% Convert COP % -> fraction of chord
f_COP = COP_fit ./ 100;

% New Formula: C_m_AC = (x_COP - x_AC) * (C_L * cos(alpha) + C_D * sin(alpha))
% This formula accounts for the Normal Force component, which generates the moment.

% Calculate the moment coefficient
Normal_Force_Coefficient = C_L_fit .* cos(alpha_range) + C_D_fit .* sin(alpha_range);
Moment_Arm_Fraction = f_COP - x_AC;
C_m_AC = Moment_Arm_Fraction .* Normal_Force_Coefficient;


% Plot pitching moment
figure;
plot(rad2deg(alpha_range), C_m_AC, 'r-', 'LineWidth',1.5);
xlabel('\alpha [deg]');
ylabel('C_{m,AC}');
title('Pitching Moment Coefficient about Aerodynamic Center');
grid on;
