function plot_results(t, x, params)

    global SIM_DATA; 
    % PLOT_RESULTS - Plot 6DoF simulation results and optionally animate
    %
    % Inputs:
    %   t      - time vector
    %   x      - state vector over time [u,v,w,p,q,r,q0,q1,q2,q3,x,y,z]'
    %   params - (optional) struct, used for animation if it contains geometry info
    
    % ---------------------- Extract states ------------------------------
    u = x(:,1); v = x(:,2); w = x(:,3);         % body velocities
    p = x(:,4); q = x(:,5); r = x(:,6);         % angular rates
    q_quat = x(:,7:10);                         % quaternions [q0 q1 q2 q3]
    pos = x(:,11:13);                           % inertial positions
        
    % ------ Convert quaternions to Euler angles (ZYX by default) --------
    % MATLAB uses quaternions as [w x y z] = [q0 q1 q2 q3]
    % quat2eul returns [yaw pitch roll] = [psi theta phi]
    eul = quat2eul(q_quat, 'ZYX');              % radians
    psi = eul(:,1); theta = eul(:,2); phi = eul(:,3);
    
    
    %% ====================== PLOTS =============================
    
    % 1. Body linear velocities
    figure('Name','Body Velocities ');
    plot(t,u,'r',t,v,'g',t,w,'b','LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Body velocities [m/s]');
    legend('u','v','w'); grid on; title('Body-frame linear velocities');

    %%%%%%%%%%%% TEST
    figure('Name', 'No sideslip velocities')
    hold on
    plot(SIM_DATA.t, SIM_DATA.u_no_sdslip, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.v_no_sdslip, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.w_no_sdslip, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Body velocities in no-sideslip frame [m/s]');
    legend('u_{no,sdslip}','v_{no,sdslip}','w_{no,sdslip}');
    grid on; title('Test if v = 0');

    %%%%%%%%%%% TEST
    % V_calc = sqrt(u.^2 + v.^2 + w.^2);
    figure('Name', 'Test velocity norm')
    hold on
    plot(SIM_DATA.t, SIM_DATA.V_no_sdslip, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.V, 'LineWidth', 1.5);
    plot(t, sqrt(u.^2 + v.^2 + w.^2), 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Body moments');
    legend('V_{sideslip}','V_{norm}','V_{sqrt,calculated}');
    grid on; title('Moments over time');

    % 2. Angular velocities
    figure('Name','Angular Velocities');
    plot(t,p/(2*pi),'r',t,q/(2*pi),'g',t,r/(2*pi),'b','LineWidth',1.5);
    xlabel('Time [s]'); ylabel('Angular rates [Hz]');
    legend('p','q','r'); grid on; title('Body angular rates');

    % 4. Euler angles
    figure('Name','Euler Angles (without roll)');
    plot(t, rad2deg([theta psi]), 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Euler angles [deg]');
    legend('\theta (pitch)','\psi (yaw)');
    grid on; title('Euler angles');

    figure('Name','Aerodynamic Angles')
    hold on
    plot(SIM_DATA.t, rad2deg(SIM_DATA.alpha), 'LineWidth', 1.5);
    plot(SIM_DATA.t, rad2deg(SIM_DATA.beta), 'LineWidth', 1.5);
    plot(SIM_DATA.t, rad2deg(SIM_DATA.alpha_equivalent), 'LineWidth', 1.5);
    %plot(SIM_DATA.t, rad2deg(SIM_DATA.no_sdslip_angle), 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Aerodynamic angles [deg]');
    legend('\alpha','\beta','\alpha_{equivalent}');%,'no-sideslip angle');
    grid on; title('Aerodynamic angles');
    
    % Aero coefficients
    figure('Name', 'Aero coefficients')
    hold on
    plot(SIM_DATA.t, SIM_DATA.CL, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.CD, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.Cm, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Aerodynamic coefficients');
    legend('C_L','C_D','C_m');
    grid on; title('Aerodynamic coefficients (around CoG)');

    % CoP vs CoG
    figure('Name', 'CoP position')
    hold on
    plot(SIM_DATA.t, SIM_DATA.COP_fraction * 100, 'LineWidth', 1.5);
    yline(params.percentage_CoG(1) * 100, '-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Position from LE in percentage');
    legend('CoP position','CoG position');
    grid on; title('CoP position variation');


    figure('Name','Aero forces')
    hold on
    plot(SIM_DATA.t, SIM_DATA.Lift_sdslip, 'LineWidth', 1.5)
    plot(SIM_DATA.t, SIM_DATA.Drag_sdslip, 'LineWidth', 1.5)
    plot(SIM_DATA.t, SIM_DATA.Normal_force, 'LineWidth', 1.5)
    plot(SIM_DATA.t, SIM_DATA.Axial_force, 'LineWidth', 1.5)
    xlabel('Time [s]'); ylabel('Aero forces');
    legend('Lift','Drag','Normal','Axial');
    grid on; title('Aero forces in wind frame and no-sideslip frames');

    figure('Name','Body aerodynamic forces')
    hold on
    plot(SIM_DATA.t, SIM_DATA.Fx_aero_b, 'LineWidth', 1.5)
    plot(SIM_DATA.t, SIM_DATA.Fy_aero_b, 'LineWidth', 1.5)
    plot(SIM_DATA.t, SIM_DATA.Fz_aero_b, 'LineWidth', 1.5)
    xlabel('Time [s]'); ylabel('Body aerodynamic forces');
    legend('F_{aero,x}','F_{aero,y}','F_{aero,z}');
    grid on; title('Aero forces in body frame');

    figure('Name','Total body forces')
    hold on
    plot(SIM_DATA.t, SIM_DATA.Fx_b, 'LineWidth', 1.5)
    plot(SIM_DATA.t, SIM_DATA.Fy_b, 'LineWidth', 1.5)
    plot(SIM_DATA.t, SIM_DATA.Fz_b, 'LineWidth', 1.5)
    xlabel('Time [s]'); ylabel('Body aerodynamic forces');
    legend('F_{body,x}','F_{body,y}','F_{body,z}');
    grid on; title('Body forces (including gravity)');

    %%%%%%%%%% TEST
    figure('Name', 'No-sideslip frame moments')
    hold on
    plot(SIM_DATA.t, SIM_DATA.Mx_no_sdslip, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.My_no_sdslip, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.Mz_no_sdslip, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Moments in no-sideslip frame');
    legend('M_{sdslip,x}','M_{sdslip,y}','M_{sdslip,z}');
    grid on; title('No-sideslip moments (Mx and Mz should be zero)');

    figure('Name', 'Body moments')
    hold on
    plot(SIM_DATA.t, SIM_DATA.Mx_b, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.My_b, 'LineWidth', 1.5);
    plot(SIM_DATA.t, SIM_DATA.Mz_b, 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Body moments');
    legend('M_x','M_y','M_z');
    grid on; title('Moments in body frame');
   
    % 3D trajectory
    figure('Name','Trajectory');
    plot3(pos(:,1), pos(:,2), pos(:,3), 'r','LineWidth',1.5);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    grid on; axis equal; title('Trajectory in inertial frame');
    set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');  % flip East and Down visually

end
