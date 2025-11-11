% Ring 1 (Front, CoM at [0.0, 0, 0]): Heavier, Wider.
M1 = 6.23*1e-3;     % kg
R_o1 = 97*1e-3;   % m (Outer Radius)
R_i1 = 96*1e-3;   % m (Inner Radius)
L1 = 54.5*1e-3;     % m (Length/Chord)
X1 = 54.5/2*1e-3;     % m (X-position of CoM)

% Ring 2 (Rear, CoM at [0.5, 0, 0]): Lighter, Narrower.
M2 = 16.5*1e-3;     % kg
R_o2 = 97*1e-3;   % m
R_i2 = 95*1e-3;   % m
L2 = 13*1e-3;     % m
X2 = 13/2*1e-3;     % m

% Execute the function
inertia_results = calculate_inertia(M1, R_o1, R_i1, L1, X1, M2, R_o2, R_i2, L2, X2);

% Display results
disp('--- Inertia Calculation Results ---');
fprintf('Total Mass (M): %.3f kg\n', inertia_results.M_total);
fprintf('Overall Center of Gravity (r_cg):\n');
disp(inertia_results.r_cg'); % Display as row vector
fprintf('\nInertia Tensor I (about overall CG):\n');
disp(inertia_results.I_total)