function [v_dot_b, omega_dot_b, q_dot, pos_dot_i] = kinematics(v_b, omega_b, q, F_b, M_b, R_bi, params)
% KINEMATICS Compute translational and rotational derivatives for 6DoF
%
% Inputs:
%   v_b      - body-frame velocity [u; v; w] (3x1)
%   omega_b  - body angular rates [p; q; r] (3x1)
%   q        - quaternion [q0; q1; q2; q3] (4x1)
%   F_b      - total force in body frame (3x1)
%   M_b      - total moment about CoM in body frame (3x1)
%   R_bi     - body->inertial rotation matrix (3x3)
%   params   - parameter struct with .m, .I, Hr_b, Hr_dot_b
%
% Outputs:
%   v_dot_b     - body-frame acceleration (3x1)
%   omega_dot_b - body-frame angular acceleration (3x1)
%   q_dot       - quaternion derivative (4x1)
%   pos_dot_i   - inertial-frame velocity (3x1)


    % 1. Acceleration in the body frame (velocity derivatives)
    v_dot_b = F_b / params.m - cross(omega_b, v_b);
    
    % 2. Angular acceleration calculation
    H_b = params.I * omega_b; % Angular momentum for the rigid body
    omega_dot_b = params.I \ (M_b - cross(omega_b, H_b));
    
    % 3. Quaternion derivative
    q_dot = 0.5 * omegaMat(omega_b) * q;
    
    % 4. Inertial velocity (for position integration)
    pos_dot_i = R_bi * v_b;

end