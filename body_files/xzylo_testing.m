function [aero,initialization] = xzylo_testing()

    %--------------------- Parameters for inertia calculation ------------------------
    % Ring 1
    M1 = 6.23*1e-3;      % kg
    R_o1 = 97*1e-3 /2;   % m (Outer Radius)
    R_i1 = 96*1e-3 /2;   % m (Inner Radius)
    L1 = 54.5*1e-3;      % m (Length/Chord)
    X1 = 54.5*1e-3 /2;   % m (X-position of CoM)
    
    % Ring 2 
    M2 = 16.5*1e-3;      % kg
    R_o2 = 97*1e-3 /2;   % m
    R_i2 = 95*1e-3 /2;   % m
    L2 = 13*1e-3;        % m
    X2 = 13.0*1e-3 /2 ;  % m
    
    %--------------- Execute the function for inertia calculation --------------------
    inertia_results = calculate_inertia(M1, R_o1, R_i1, L1, X1, M2, R_o2, R_i2, L2, X2);

    aero.CoG = inertia_results.r_cg; % From the leading edge
    aero.percentage_CoG = aero.CoG./L1  *1.1;
    aero.percentage_AC = [0.25; 0; 0]; % 25% of the chord is the aerodynamic center
    aero.I   = inertia_results.I_total;  % inertia matrix in body frame
    aero.m   = inertia_results.M_total;  % mass [kg]

    aero.b = R_o1;   % span = diameter
    aero.c = L1;     % chord = distance from LE to TE
    aero.S = aero.b*aero.c; % reference area
    aero.aspect_ratio = aero.b/aero.c; % span/chord

    aero.rho = 1.225;
    aero.V_wind_i = [0; 0; 0]; % wind velocity in inertial frame [m/s]

    aero.r_offset = (aero.percentage_CoG - aero.percentage_AC).*[aero.c; aero.b/2; aero.b/2];
    aero.g = 9.81;
    
    %----------------------- Aerodynamic coefficient functions --------------------------
    aerodynamics_xzylo      % run aerodynamic file for interpolation of data
    aero.CL =  @(angle) interp1(alpha_rad_CL, C_L_sym_smooth, angle, 'pchip', 'extrap');            
    aero.CD =  @(angle) interp1(alpha_rad_CD, C_D_sym_smooth, angle, 'pchip', 'extrap');
    aero.CoP_frac = @(angle) interp1(alpha_rad_COP, COP_sym_smooth, angle, 'pchip', 'extrap')./ 100;
    aero.Cm =  @(angle) (aero.percentage_CoG(1) - aero.CoP_frac(angle)) * (aero.CL(angle) * cos(angle) + aero.CD(angle) * sin(angle));

    % Optional external force/moment (written in body frame)
    aero.Fext = @(t) (t >= 1 && t <= 2.0) * [0; 0; 0];
    aero.Mext = @(t) (t >= 0 && t <= 30) * [0; 0; 0];

    % ------------------------- Initialization states ------------------------------------
    initialization.launch_angle = 5; % launch angle in degrees
    initialization.V_mag = 20; % Magnitude of the launch velocity
    initialization.Omega_mag = 40; % Rotational speed at launch [RPS]

    initialization.v0 = [initialization.V_mag,  0  , 0]; % initial velocity vector (u,v,w)_0   [m/s]
    initialization.omega0 = [2*pi*initialization.Omega_mag, 0, 0]; % initial rotational velocity vector (p,q,r)_0  [rad/s]

    initialization.euler0 = [0*pi/180, initialization.launch_angle*pi/180, 0*pi/180]; %[yaw pitch roll];   % [psi theta phi]
    initialization.quat0  = eul2quat(initialization.euler0);   % returns [w x y z]

    initialization.pos0 = [0 0 -10]; % initial position in inertial frame [m]

    initialization.tf = 30; %maximum simulation time
   
end