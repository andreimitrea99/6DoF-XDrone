                                                                                                                                                                                                           %% ===============================
%% MAIN.M - 6DoF SIMULATION
%% ===============================
close all
clear, clc

%% --- Global Data Storage Declaration ---
global SIM_DATA; 
% Initialize struct to store data
SIM_DATA = struct('t', [], 'V', [], 'V_no_sdslip', [], ...
    ... % Velocities in no-sideslip rotated frame
    'u_no_sdslip', [], 'v_no_sdslip', [], 'w_no_sdslip', [], ...
    ... % Aero Forces in Body Frame
    'Fx_aero_b', [], 'Fy_aero_b', [], 'Fz_aero_b', [], ...
    ... % Aero Forces in wind frame and rotated by alpha_equivalent for no-sideslip frame
    'Lift_sdslip', [], 'Drag_sdslip', [], 'Normal_force', [], 'Axial_force', [], ...
    ... % Forces in body frame (complete, including gravity)
    'Fx_b', [], 'Fy_b', [], 'Fz_b', [], ...
    ... % Total Moments in Body Frame
    'Mx_b', [], 'My_b', [], 'Mz_b', [], ...
    ... % Moments computed in the no-sideslip frame (x and z components should be zero)
    'Mx_no_sdslip', [], 'My_no_sdslip', [], 'Mz_no_sdslip', [], ...
    ... % Aero oefficients
    'CL', [], 'CD', [], 'Cm', [], ...
    ... % Angles / derived angles
    'alpha', [], 'beta', [], 'no_sdslip_angle', [], 'alpha_equivalent', [], ...
    ... % Centre of Pressure fraction location
    'COP_fraction', [] ...
);

%% --- INITIALIZATION ---
% =================================
[params,initial] = xzylo_testing();
% =================================

x0 = [initial.v0 initial.omega0 initial.quat0 initial.pos0]';
params.x0 = x0;

% --- Simulation setup ---
tspan = [0 initial.tf];  % total time

% --- Plotting setup ---
params.radius_visualization = 5;
params.live_plotting_residuals = false; 

%% --- INTEGRATION WITH LIVE PLOT ---

if params.live_plotting_residuals == false
    % Simple plotting without residuals (faster)
    options = odeset('RelTol',1e-6,'AbsTol',1e-9,'Events', @hit_ground);
else
    % Plotting with residuals of the x states (slow)
    fig = figure('Name','Live dx evolution','NumberTitle','off');
    tiledlayout(fig, 4, 4, 'TileSpacing','tight'); 
    sgtitle('Time evolution of dx components')
    
    % Store parameters & plot handles for access inside OutputFcn
    setappdata(fig, 'params', params);
    setappdata(fig, 'x0', x0);
    
    % Create line objects for speed (no replotting overhead)
    for i = 1:13
        ax(i) = nexttile;
        hold(ax(i), 'on');
        title(ax(i), sprintf('dx(%d)', i));
        xlabel(ax(i), 't [s]');
        ylabel(ax(i), sprintf('dx_%d', i));
        grid(ax(i), 'on');
        h(i) = plot(ax(i), NaN, NaN, 'b.');
    end
    
    setappdata(fig, 'h', h);
    setappdata(fig, 'ax', ax);
    
    options = odeset('RelTol',1e-6,'AbsTol',1e-9,'Events',@hit_ground,...
        'OutputFcn', @(t,y,flag) live_dx_plot(t,y,flag,fig));
end

% ========================= INTEGRATOR ===============================
[t, x] = ode45(@(t,x) sixdof_wrapper(t,x,params), tspan, x0, options);
% ====================================================================

%% POST-PROCESSING
plot_results(t,x,params);
plot_animation(t, x, params);
%visualization_XZylo_orientation(params);