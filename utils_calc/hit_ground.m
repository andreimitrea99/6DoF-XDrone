function [value, isterminal, direction] = hit_ground(t,x)
% Stops the simulation when z position <= 0
%
% Inputs:
%   t - current time
%   x - current state vector
%
% Outputs:
%   value - the function we monitor (stop when value = 0)
%   isterminal - 1 to stop the integrator
%   direction - 0 means all zero-crossings

z = x(13);   % z position in inertial frame
value = z;   % integration stops when value = 0
isterminal = 1;  % stop the integration
direction = 1;  % only detect decreasing zeros (falling through z=0)
end
