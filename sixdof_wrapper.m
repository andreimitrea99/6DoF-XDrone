function dx = sixdof_wrapper(t, x, params)
    
    % Unpack
    v_b = x(1:3);
    omega_b = x(4:6);
    q = x(7:10);
    pos_i = x(11:13);
    
    % Normalize quaternion and use in dynamics
    q = quatNorm(q);
    
    % Rotation matrices for rotating body frame
    R_bi = quat2rotm(q');    % body -> inertial
    R_ib = R_bi';            % inertial -> body
    
    % Dynamics
    [F_b, M_b] = dynamics(t, v_b, R_ib, params); 
    
    % Kinematics
    [v_dot_b, omega_dot_b, q_dot, pos_dot_i] = kinematics(v_b, omega_b, q, F_b, M_b, R_bi, params);
    
    % Pack
    dx = [v_dot_b; omega_dot_b; q_dot; pos_dot_i];
end
