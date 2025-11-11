function [aero,initialization] = soccer_ball2()

    % --- 1. Physical Dimensions and Mass Properties ---
    
    % Physical Dimensions (based on regulation size, averages)
    L_major_axis = 0.28;        % Major axis length (tip-to-tip) [m]
    D_max_diameter = 0.1725;      % Maximum diameter [m]
    
    % Mass and Inertia (approximated for a 0.400 kg prolate spheroid)
    aero.m = 0.411;             % Mass [kg]
    
    % Inertia Matrix (I_total in body frame [kg*m^2])
    I_roll  = 0.00194;          % Ixx (Longitudinal MOI, along the spin axis)
    I_pitch = 0.00321;          % Iyy (Transverse MOI)
    I_yaw   = 0.00321;          % Izz (Transverse MOI)
    aero.I = diag([I_roll, I_pitch, I_yaw]);
    
    % Center of Gravity (CoG) location
    % A well-balanced football has its CoG at the geometric center.
    aero.CoG = [0; 0; 0];       % CoG is at the body origin (center)
    aero.percentage_CoG = 0.5;  % 50% of L_major_axis
    
    % Reference Dimensions for Aerodynamics
    aero.L = L_major_axis;      % Major length (used for moment calculations)
    aero.D = D_max_diameter;    % Max diameter
    aero.S = 0.0233; % Reference Area: Max cross-sectional area [m^2]
    aero.aspect_ratio = aero.L / aero.D; % approx 1.65
            
    aero.b = D_max_diameter;     % span = diameter
    aero.c = L_major_axis;       % chord = distance from LE to TE

    % Atmospheric and Environmental
    aero.rho = 1.225;           % Air density at sea level [kg/m^3]
    aero.g = 9.81;              % Gravity [m/s^2]
    aero.V_wind_i = [0; 0; 0];  % Initial wind vector [m/s]
    
    % --- 2. Aerodynamic Centers and Offsets ---
    
    % Aerodynamic Center (AC) relative to CoG:
    % AC is set at 35% of L (0.35 * 0.28m = 0.098m from nose)
    % CoG is at 50% of L (0.50 * 0.28m = 0.140m from nose)
    % Offset (from CoG) = AC position - CoG position = 0.098m - 0.140m = -0.042m
    
    aero.percentage_AC = 0.35;
    aero.r_offset = [(aero.percentage_AC-aero.percentage_CoG)*L_major_axis; 0; 0]; % Aerodynamic Center location [m] relative to CoG (4.2 cm REARWARD)
    
    % --- 3. Aerodynamic Coefficient Polynomials (Based on Rae, 2003, and related literature) ---
    
    aero.CD = @(alpha, beta) 0.14*cos(alpha);
    aero.CL = @(alpha, beta) 0.8*alpha;
    aero.CY = @(alpha, beta) 0.28*sin(1.5*beta);
    % Moment Coefficient 
    aero.Cm = @(alpha, beta) -0.165*sin(2*alpha);
    aero.Cl = @(alpha, beta) 0*alpha;
    aero.Cn = @(alpha, beta) 0*beta;
    
    % Optional external force/moment
    aero.Fext = @(t) (t >= 1 && t <= 2) * [0; 0; 0];
    aero.Mext = @(t) (t >= 1 && t <= 1.1) * [0; 0; 0];
    
    % --- Initial State ---
    
    % Initialization based on "long bomb" paper
    initialization.launch_angle = 30; % in degrees from WAGNER
    initialization.v0 = [27.4*cosd(initialization.launch_angle) 0 27.4*sind(initialization.launch_angle)]; % initial velocity vector (u,v,w)_0
    initialization.omega0 = [62.8 0 0]; % initial rotational velocity vector (p,q,r)_0 (omega_spin = 62.8 rad/s)
    initialization.euler0 = [0, -initialization.launch_angle*pi/180, 0]; %[yaw pitch roll];   % [psi theta phi]
    initialization.quat0  = eul2quat(initialization.euler0);   % returns [w x y z]
    initialization.pos0 = [0 0 2]; % initial position in inertial frame
    initialization.tf = 100; %maximum simulation time
   
end
