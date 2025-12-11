% =========================================================================
% GNSS TRILATERATION SOLVER (Newton-Raphson Method)
% =========================================================================
% This script demonstrates how to solve for a receiver's position (x,y,z)
% and clock bias (b) using pseudoranges from 4 satellites.
%
% It generates a random "True" scenario, calculates what the satellites
% "see", and then forces the receiver to figure out where it is starting 
% from a random, incorrect guess.
% =========================================================================

clear; clc; close all;

%% 1. CONSTANTS AND SETUP
c = 299792458; % Speed of light (m/s)

% Earth radius approx (meters) for generating realistic coordinates
R_earth = 6371000; 
% Satellite orbital radius approx (meters) ~20,000km altitude
R_sat = 26560000; 

fprintf('------------------------------------------------------------\n');
fprintf('GNSS POSITIONING SIMULATION\n');
fprintf('------------------------------------------------------------\n');

%% 2. GENERATE TRUE SCENARIO (The "Hidden" Truth)

% A. Generate Random True Receiver Position (ECEF Coordinates)
% We pick a random spot roughly on Earth's surface
theta = rand * 2 * pi;         % Random longitude
phi = (rand - 0.5) * pi;       % Random latitude

% Starting off with zeros to make the math easier
true_pos = R_earth * [cos(phi)*cos(theta), cos(phi)*sin(theta), sin(phi)];

% B. Generate Random Receiver Clock Bias 
% (Receivers have cheap clocks, so they have an offset from GPS time)
% Let's say the clock is off by ~100 microseconds (approx 30,000 meters)
true_bias_distance = (rand - 0.5) * 30000; 

fprintf('TRUE Position (Hidden):  [%10.2f, %10.2f, %10.2f]\n', true_pos);
fprintf('TRUE Clock Bias (Hidden): %.2f meters\n\n', true_bias_distance);

% C. Generate 4 Satellites (Known Positions)
% We place them in a "sky" around the receiver to ensure decent geometry.
% (In reality, these are read from the Navigation Message / Ephemeris)
% Satellites on the row, xyz in columns
sat_pos = zeros(4, 3);
% Sat 1: Generally overhead
sat_pos(1,:) = true_pos + [0, 0, R_sat - R_earth]; 
% Sat 2,3,4: Spread out North, East, West
sat_pos(2,:) = true_pos + [1e7, 1e7, 1e7];
sat_pos(3,:) = true_pos + [-1e7, 1e7, 1e7];
sat_pos(4,:) = true_pos + [0, -1e7, 1e7];

% D. Generate "Measured" Pseudoranges
% This is the raw data a receiver would get from the hardware.
% Pseudorange = True Distance + Clock Bias Distance
pseudoranges = zeros(4, 1);
for i = 1:4
    true_dist = norm(sat_pos(i,:) - true_pos);
    pseudoranges(i) = true_dist + true_bias_distance;
end

%% 3. THE SOLVER INITIALIZATION

% We start with a "Blind Guess".
% Ideally, receivers start at the center of the Earth (0,0,0) or the last 
% known position. Here, we offset the true position by a large random amount
% (e.g., 500km error) to show the convergence power.
random_offset = (rand(1,3) - 0.5) * 1000000; % +/- 500km error
curr_pos = true_pos + random_offset;

curr_bias = 0; % We assume 0 clock error initially

fprintf('INITIAL GUESS (Random):  [%10.2f, %10.2f, %10.2f]\n', curr_pos);
fprintf('Initial Position Error:  %.2f meters\n', norm(curr_pos - true_pos));
fprintf('------------------------------------------------------------\n');
fprintf('Starting Iteration Loop (Linearization)...\n');
fprintf('%-5s | %-15s | %-15s | %-15s\n', 'Iter', 'Pos correction', 'Bias correction', 'Current Error');
fprintf('------------------------------------------------------------\n');

%% 4. THE ITERATION LOOP (Linear Least Squares)

function [H, dP, bias] = jH(H, dP, bias, pseudoranges, r_pos, s_positions, s_id)
    % 1. Calculate Geometric Distance from Current Guess to Satellite
    % r = sqrt((xs-x)^2 + (ys-y)^2 + (zs-z)^2)
    dx = r_pos(1) - s_positions(s_id, 1);
    dy = r_pos(2) - s_positions(s_id, 2);
    dz = r_pos(3) - s_positions(s_id, 3);
    geometric_dist = sqrt(dx^2 + dy^2 + dz^2);
    
    % 2. Calculate "Modeled" Pseudorange
    % What the pseudorange *should* be if we were actually at 'curr_pos'
    modeled_pr = geometric_dist + bias;
    
    % 3. Calculate the Residual (Observation - Model)
    dP(s_id) = pseudoranges(s_id) - modeled_pr;
    
    % 4. Fill the Jacobian (Geometry Matrix) H
    % The derivative of the distance equation results in unit vectors.
    % H = [ -UnitVector_X, -UnitVector_Y, -UnitVector_Z, 1 ]
    
    H(s_id, 1) = -dx / geometric_dist; % partial deriv wrt x
    H(s_id, 2) = -dy / geometric_dist; % partial deriv wrt y
    H(s_id, 3) = -dz / geometric_dist; % partial deriv wrt z
    H(s_id, 4) = 1;  
end

max_iter = 20;
tolerance = 1e-4; % Stop when correction is less than 0.1 mm

for iter = 1:max_iter
    % A. Pre-allocate Jacobian Matrix (H) and Residual Vector (dP)
    H = zeros(4, 4);
    dP = zeros(4, 1);
    
    for i = 1:4
        [H, dP, curr_bias] = jH(H, dP, curr_bias, pseudoranges, curr_pos, sat_pos, i);
    end
    
    % B. Solve for corrections
    % We have the linear system: dP = H * dx
    % We solve for dx: dx = inv(H) * dP
    % Using MATLAB's '\' operator for stability:
    % '\' left matrix divide i.e. 
    % find unique solutions (x) to Ax = b
    correction = H \ dP;
    
    % C. Update the Estimates
    curr_pos(1) = curr_pos(1) - correction(1);
    curr_pos(2) = curr_pos(2) - correction(2);
    curr_pos(3) = curr_pos(3) - correction(3);
    curr_bias   = curr_bias   - correction(4);
    
    % Calculate magnitude of position correction for convergence check
    pos_correction_mag = norm(correction(1:3));
    current_error = norm(curr_pos - true_pos);
    
    fprintf('%-5d | %-15.4f | %-15.4f | %-15.4f\n', iter, pos_correction_mag, correction(4), current_error);
    
    % D. Check Convergence
    if norm(correction(1:3)) < tolerance
        fprintf('------------------------------------------------------------\n');
        fprintf('CONVERGED in %d iterations.\n', iter);
        break;
    end
end

%% 5. FINAL RESULTS DISPLAY

fprintf('\n=== FINAL RESULTS ===\n');
fprintf('Parameter       | %15s | %15s | %15s\n', 'Calculated', 'True', 'Residual');
fprintf('----------------|-----------------|-----------------|----------------\n');
fprintf('X Coordinate    | %15.3f | %15.3f | %15.3e\n', curr_pos(1), true_pos(1), abs(curr_pos(1)-true_pos(1)));
fprintf('Y Coordinate    | %15.3f | %15.3f | %15.3e\n', curr_pos(2), true_pos(2), abs(curr_pos(2)-true_pos(2)));
fprintf('Z Coordinate    | %15.3f | %15.3f | %15.3e\n', curr_pos(3), true_pos(3), abs(curr_pos(3)-true_pos(3)));
fprintf('Clock Bias (m)  | %15.3f | %15.3f | %15.3e\n', curr_bias, true_bias_distance, abs(curr_bias-true_bias_distance));