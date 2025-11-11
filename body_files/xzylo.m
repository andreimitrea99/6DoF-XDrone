function [aero,initialization] = xzylo()

    aerodynamics_XZYLO

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

    aero.CoG =  inertia_results.r_cg; % From the leading edge
    aero.percentage_CoG =  aero.CoG./L1;
    aero.I   = inertia_results.I_total;  % inertia matrix in body frame
    aero.m   = inertia_results.M_total;  % mass [kg]

    aero.inner_r = 95*1e-3;
    aero.outer_r = 97*1e-3;
    aero.b = aero.outer_r;      % span = diameter
    aero.c = L1;                % chord = distance from LE to TE
    aero.S   = aero.b* aero.c;
    aero.aspect_ratio = aero.b/aero.c; %span/chord


    aero.rho = 1.225;
    aero.V_wind_i = [0; 0; 0];
    
    % VERIFY AERODYNAMICS_XZYLO IF MOMENT IS WRITTEN ABOUT COG OR AC
    aero.r_offset = [1.5*1e-3; 0; 0];
    aero.g = 9.81;
    
    % Aerodynamic coefficient functions
    % aero.CD = @(alpha,beta) polyval(p_CD, alpha);
    % aero.CL = @(alpha,beta) polyval(p_CL, alpha);
    % aero.CY = @(alpha,beta) polyval(p_CL, beta);
    % aero.Cl = @(alpha,beta) 0*alpha;
    % aero.Cm = @(alpha,beta) -polyval(p_Cm, alpha);
    % aero.Cn = @(alpha,beta) 0*alpha;
    alpha_min = rad2deg(-25);
    alpha_max = rad2deg(25); % Define maximum angle of attack
    aero.CL = @(alpha) polyval(p_CL, max(min(alpha, alpha_max), alpha_min));
    aero.CD = @(alpha) polyval(p_CD, max(min(alpha, alpha_max), alpha_min));
    aero.Cm = @(alpha) -polyval(p_Cm, max(min(alpha, alpha_max), alpha_min));
    
    % Optional external force/moment
    aero.Fext = @(t) (t >= 1 && t <= 2) * [0; 0; 0];
    aero.Mext = @(t) (t >= 1 && t <= 1.1) * [0; 0; 0];

    % --- Initial State ---
    initialization.launch_angle = 12; % in degrees from WAGNER
    initialization.v0 = [16*cosd(initialization.launch_angle) 0 16*sind(initialization.launch_angle)]; % initial velocity vector (u,v,w)_0
    initialization.omega0 = [40 0 0]; % initial rotational velocity vector (p,q,r)_0
    initialization.euler0 = [0, -initialization.launch_angle*pi/180, 0]; %[yaw pitch roll];   % [psi theta phi]
    initialization.quat0  = eul2quat(initialization.euler0);   % returns [w x y z]

    initialization.pos0 = [0 0 1.5]; % initial position in inertial frame

    initialization.tf = 100; %maximum simulation time
   
end
