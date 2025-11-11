function [F_b, M_b] = dynamics(t, v_b, R_ib, params)
% DYNAMICS Compute total forces and moments for 6DoF rigid body
%
% Includes a despun (non-rotating) frame to calculate aerodynamics more cleanly.
% The despun frame shares the same x-axis as the body, but has zero roll about x.
%
% Inputs:
%   v_b     - body-frame velocity [u; v; w]
%   R_ib    - rotation matrix inertial->body
%   params  - struct with .rho, .S, .m, .g, etc.
%   t       - current time (s)
%
% Outputs:
%   F_b     - total force in body frame
%   M_b     - total moment in body frame

    global SIM_DATA;

    % Compute velocity relative to wind (in body frame)
    if isfield(params,'V_wind_i')
        V_rel_b = v_b - R_ib * params.V_wind_i;
    else
        V_rel_b = v_b; % no wind
    end

    u_r = V_rel_b(1); 
    v_r = V_rel_b(2); 
    w_r = V_rel_b(3);
    V = norm(V_rel_b);

    if V < 1e-8
        alpha = 0; beta = 0;
    else
        alpha = atan2(w_r, u_r);   % AoA
        beta  = asin(v_r / V);     % sideslip
    end

    % Compute angle to rotate about body x so lateral component becomes zero
    if abs(w_r) < 1e-15 && abs(v_r) < 1e-15
        no_sdslip_angle = 0;
    else
        no_sdslip_angle = atan2(v_r, w_r);   % psi = atan2(v,w)
    end

    % Rotation matrix for sdslip about x-axis
    R_no_sdslip = [1 0 0
                   0 cos(no_sdslip_angle) -sin(no_sdslip_angle)
                   0 sin(no_sdslip_angle) cos(no_sdslip_angle)];

    % Express velocity in non-sideslip frame (should have v ≈ 0)
    V_no_sdslip_frame = R_no_sdslip * V_rel_b;
    u_no_sdslip = V_no_sdslip_frame(1);
    v_no_sdslip = V_no_sdslip_frame(2);
    w_no_sdslip = V_no_sdslip_frame(3);

    % Total angle of attack
    if V < 1e-8
        alpha_equivalent = 0;
    else
        alpha_equivalent = atan2(w_no_sdslip, u_no_sdslip);
    end

    % Dynamic pressure
    qbar = 0.5 * params.rho * V^2;

    % Aerodynamic coefficients (based on total AoA only for axisymmetric)
    C_D = params.CD(alpha_equivalent);
    C_L = params.CL(alpha_equivalent);

    % Forces in no-sideslip rotated frame (convention: drag opposes x_th, lift opposes z_th)
    F_wind_frame = qbar * params.S * [-C_D; 0; -C_L];

    % Forces must be rotated by alpha_equivalent to go in no-sideslip frame
    R_alpha_total = [cos(alpha_equivalent) 0 -sin(alpha_equivalent)
                     0 1 0
                     sin(alpha_equivalent) 0 cos(alpha_equivalent)];

    % Transformation from lift/drag to normal/axial
    F_aero_no_sdslip = R_alpha_total * F_wind_frame;
        
    % Rotate forces back to body frame
    F_aero_body = R_no_sdslip' * F_aero_no_sdslip;
    %F_aero_b = R_ib*[0;0; -0.22298130];%*(1+cos(2*4*pi*t));     % only for testing 

    % Gravity (in body frame)
    F_g = params.m * [0; 0; params.g]; % inertial frame
    F_g_b = R_ib * F_g; % gravity in body frame
    
    % Forces in body frame including gravity, without external forces
    F_b_no_external = F_aero_body + F_g_b;

    % External forces addition (if any)
    if isfield(params,'Fext') && ~isempty(params.Fext)
        F_b = F_b_no_external + params.Fext(t);
    else
        F_b = F_b_no_external;
    end
    
    % --------------------- Aerodynamic moments ---------------------------
    C_l = 0; % roll moment negligible for axisymmetric
    C_m = params.Cm(alpha_equivalent);
    C_n = 0; % 0 because we look in the no-sideslip frame

    M_no_sdslip = qbar * params.S * [params.b * C_l
                                     params.c * C_m
                                     params.b * C_n];

    % Rotate the moments around x axis to go back to body frame
    M_b_no_external = R_no_sdslip' * M_no_sdslip;

    % M_aero_AC_b = R_sdslip' * M_sdslip;

    % % Account for possible offset between AC and CoM
    % if isfield(params,'r_offset')
    %     M_b_no_external = M_aero_AC_b + cross(params.r_offset, F_aero_body); %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % else
    %     M_b_no_external = M_aero_AC_b;
    % end
    
    % External moments addition (if any)
    if isfield(params,'Mext') && ~isempty(params.Mext)
        M_b = M_b_no_external + params.Mext(t);
    else
        M_b = M_b_no_external;
    end

    % ------------------ LOGGING DATA ------------------
    SIM_DATA.t(end+1)  = t;
    SIM_DATA.V(end+1)  = V;
    SIM_DATA.V_no_sdslip(end+1) = norm(V_no_sdslip_frame); % should be equal to V from above
    SIM_DATA.u_no_sdslip(end+1) = u_no_sdslip;
    SIM_DATA.v_no_sdslip(end+1) = v_no_sdslip; % should be zero
    SIM_DATA.w_no_sdslip(end+1) = w_no_sdslip;

    SIM_DATA.Lift_sdslip(end+1) = -F_wind_frame(3); % I took lift and drag as positive values
    SIM_DATA.Drag_sdslip(end+1) = -F_wind_frame(1);
    SIM_DATA.Normal_force(end+1) = -F_aero_no_sdslip(3);
    SIM_DATA.Axial_force(end+1) = -F_aero_no_sdslip(1);

    SIM_DATA.Fx_aero_b(end+1) = F_aero_body(1); 
    SIM_DATA.Fy_aero_b(end+1) = F_aero_body(2);
    SIM_DATA.Fz_aero_b(end+1) = F_aero_body(3);

    SIM_DATA.Fx_b(end+1) = F_b(1); 
    SIM_DATA.Fy_b(end+1) = F_b(2);
    SIM_DATA.Fz_b(end+1) = F_b(3);
    
    SIM_DATA.Mx_no_sdslip(end+1) = M_no_sdslip(1); % Should be zero 
    SIM_DATA.My_no_sdslip(end+1) = M_no_sdslip(2);
    SIM_DATA.Mz_no_sdslip(end+1) = M_no_sdslip(3); % Should be zero

    SIM_DATA.Mx_b(end+1) = M_b(1);
    SIM_DATA.My_b(end+1) = M_b(2);
    SIM_DATA.Mz_b(end+1) = M_b(3); 

    SIM_DATA.CL(end+1) = C_L; 
    SIM_DATA.CD(end+1) = C_D;

    SIM_DATA.Cm(end+1) = C_m;

    SIM_DATA.alpha(end+1) = alpha;
    SIM_DATA.beta(end+1)  = beta;
    SIM_DATA.no_sdslip_angle(end+1) = no_sdslip_angle;
    SIM_DATA.alpha_equivalent(end+1) = alpha_equivalent;

    SIM_DATA.COP_fraction(end+1) = params.CoP_frac(alpha_equivalent);



end
