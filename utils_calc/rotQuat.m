function [R_bi, R_ib] = rotQuat(q)
% Rotational matrix function call

%gives the rotational matrix from quaternions
%R_bi : body to inertial
%R_ib : inertial to body
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    R_bi = [ ...
        q0^2+q1^2-q2^2-q3^2, 2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
        2*(q1*q2 + q0*q3),   q0^2-q1^2+q2^2-q3^2, 2*(q2*q3 - q0*q1);
        2*(q1*q3 - q0*q2),   2*(q2*q3 + q0*q1),   q0^2-q1^2-q2^2+q3^2];
%Invers of a rotational matrix for quaternions is just the transpose
    R_ib = R_bi';
end