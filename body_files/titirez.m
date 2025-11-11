function [params,initial] = titirez()
    params.m   = 0.1;        % mass [kg]
    params.I   = diag([0.1 0.1 0.1]);  % inertia matrix in body frame
    params.rho = 1.225;
    params.S   = 0.1; 
    params.b = 0.3; 
    params.c = 0.1;
    params.V_wind_i = [0; 0; 0];
    
    params.r_offset = [-0.1; 0; 0];
    params.g = 0;% 9.81;
    
    % Aerodynamic coefficient functions
    params.CD = @(alpha,beta) 0*alpha; %0.05 + 0.5*alpha^2;
    params.CL = @(alpha,beta) 0*pi*alpha;
    params.CY = @(alpha,beta) -0*pi*beta;
    params.Cl = @(alpha,beta) -0*alpha;
    params.Cm = @(alpha,beta) -0*alpha;
    params.Cn = @(alpha,beta) -0*alpha;
    
    % Optional external force/moment
    params.Fext = @(t) (t >= 0 && t <= 2) * [10; 0; 0];
    params.Mext = @(t) (t >= 3 && t <= 7) * [0; 0; 0];
    
    % --- Initial State ---
    initial.v0 = [0 0 0]; % initial velocity vector (u,v,w)_0
    initial.omega0 = [0 0 10]; % initial rotational velocity vector (p,q,r)_0
    initial.quat0 = [1 0 0 0];  % unit quaternion
    initial.pos0 = [0 0 10]; % initial position in inertial frame

    initial.tf = 10; %maximum simulation time
end