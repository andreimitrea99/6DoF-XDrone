function [aero,initialization] = xzylo_straight()

    % INERTIA CALCULATIONS
    % Ring 1
    M1 = 6.23*1e-3;     % kg
    R_o1 = 97*1e-3;     % m (Outer Radius)
    R_i1 = 96*1e-3;     % m (Inner Radius)
    L1 = 54.5*1e-3;     % m (Length/Chord)
    X1 = 54.5/2*1e-3;   % m (X-position of CoM)
    
    % Ring 2 
    M2 = 16.5*1e-3;   % kg
    R_o2 = 97*1e-3;   % m
    R_i2 = 95*1e-3;   % m
    L2 = 13*1e-3;     % m
    X2 = 13/2*1e-3;   % m
    
    % Execute the function
    inertia_results = calculate_inertia(M1, R_o1, R_i1, L1, X1, M2, R_o2, R_i2, L2, X2);

    aero.CoG = inertia_results.r_cg; % From the leading edge
    aero.I   = inertia_results.I_total;  % inertia matrix in body frame
    aero.m   = inertia_results.M_total;  % mass [kg]

    aero.inner_r = 95*1e-3;
    aero.outer_r = 97*1e-3;
    aero.S   = 0.1; 
    aero.b = aero.outer_r;      % span = diameter
    aero.c = 54.5*1e-3;         % chord = distance from LE to TE
    aero.aspect_ratio = aero.b/aero.c; %span/chord

    aero.rho = 1.225;
    aero.V_wind_i = [0; 0; 0];
    
    aero.r_offset = [0; 0; 0];%[-1.5*1e-3; 0; 0];
    aero.g = 9.81;
    
    % Aerodynamic coefficient functions
    aero.CD = @(alpha,beta) 0.0*alpha;%0.005 + 0.5*alpha^2;
    aero.CL = @(alpha,beta) 0.0*alpha;
    aero.Cl = @(alpha,beta) 0*alpha;
    aero.Cm = @(alpha,beta) 0*alpha;
    aero.Cn = @(alpha,beta) 0*beta;
    
    % Optional external force/moment
    aero.Fext = @(t) (t >= 1 && t <= 2) * [0; 0; 0];
    aero.Mext = @(t) (t >= 1 && t <= 1.1) * [0; 0; 0];

    % --- Initial State ---
    initialization.v0 = [20 0 0]; % initial velocity vector (u,v,w)_0
    initialization.omega0 = [10 0 0]; % initial rotational velocity vector (p,q,r)_0
    initialization.quat0 = [1 0 0 0];  % unit quaternion
    initialization.pos0 = [0 0 10]; % initial position in inertial frame

    initialization.tf = 100; %maximum simulation time
   
end
