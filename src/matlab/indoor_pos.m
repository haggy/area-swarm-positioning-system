% =========================================================================
% INDOOR POSITIONING SYSTEM (IPS) COMPARISON
% =========================================================================
% This script visualizes the mathematical process to find indoor positions
% INDOOR (BLE): 2D, RSSI (Log-Distance), Weighted Trilateration.
% =========================================================================

clear; clc; close all;

% Create Figure
figure('Name', 'GNSS vs Indoor Positioning', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 600]);


%% ========================================================================
%  INDOOR IPS SIMULATION (2D, RSSI-Based, Weighted)
% ========================================================================
subplot(1, 2, 2);
hold on; grid on; axis equal;
title({'Indoor IPS (Local)'; 'Method: RSSI (Log-Distance)'; 'Unknowns: x, y (Weighted)'});
xlabel('X (m)'); ylabel('Y (m)');
xlim([-2 22]); ylim([-2 22]);

% --- 2.1 Physics Constants ---
% Log-Normal Path Loss Model parameters
Tx_Power_1m = -60;  % dBm (Signal strength at 1 meter)
N_exponent = 2.5;   % Path loss exponent (2.0=Free Space, 4.0=Cluttered)
Noise_Std = 4;      % Standard Deviation of Noise (dBm) - Very noisy indoors!

% --- 2.2 Setup Geometry (20m x 20m Room) ---
indoor_true_pos = [12, 8]; % User is at (12, 8)
rectangle('Position', [0, 0, 20, 20], 'EdgeColor', 'k', 'LineWidth', 2); % The Room
plot(indoor_true_pos(1), indoor_true_pos(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'True Pos');

% Beacons (Corners)
beacons = [
    0, 0;
    20, 0;
    20, 20;
    0, 20
];

% Plot Beacons
plot(beacons(:,1), beacons(:,2), 'b^', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'BLE Beacons');

% --- 2.3 Measurements (RSSI with Channel Diversity) ---
% The whitepaper suggests "Channel Diversity": reading multiple frequencies 
% to reduce fading. We simulate this by averaging 3 noise samples per beacon.
indoor_dist_est = zeros(4, 1);
indoor_weights = zeros(4, 1);

for i = 1:4
    true_d = norm(beacons(i,:) - indoor_true_pos);
    
    % Simulate 3 readings (Channel 37, 38, 39) and average them
    % rssi_samples = zeros(1,3);
    % for ch = 1:3
    %     noise = randn * Noise_Std; 
    %     rssi_samples(ch) = Tx_Power_1m - 10 * N_exponent * log10(true_d) + noise;
    % end
    % avg_rssi = mean(rssi_samples);
    avg_rssi = (90 - randi(20) * randn) * -1;
    fprintf('avg_rssi=%.5f\n', avg_rssi);
    
    % INVERSE PHYSICS: Convert RSSI back to Distance
    % d = 10 ^ ((TxPower - RSSI) / (10 * n))
    est_d = 10 ^ ((Tx_Power_1m - avg_rssi) / (10 * N_exponent));
    indoor_dist_est(i) = est_d;
    
    % WEIGHT CALCULATION (Whitepaper Concept)
    % We trust close beacons more than far beacons because the log-curve 
    % makes far distance estimates extremely sensitive to dBm noise.
    % Weight = 1 / Distance^2
    indoor_weights(i) = 1 / (est_d^2); 
    
    % Visualization: Draw the estimated distance circles
    fprintf('est_d=%.5f\n', est_d);
    viscircles(beacons(i,:), est_d, 'Color', [0.1 0.1 0.8], 'LineStyle', '--', 'LineWidth', 0.5);
end

% --- 2.4 Solver (Weighted Gauss-Newton 2D) ---
indoor_est = [10, 10]; % Guess center of room (no clock bias needed!)

for iter = 1:10
    H = zeros(4, 2);
    dP = zeros(4, 1);
    W = diag(indoor_weights); % Weight Matrix
    
    for i = 1:4
        geo_dist = norm(indoor_est - beacons(i,:));
        dP(i) = indoor_dist_est(i) - geo_dist;
        
        % Jacobian: 2D Geometry only (No clock column)
        H(i, 1) = (indoor_est(1) - beacons(i,1)) / geo_dist;
        H(i, 2) = (indoor_est(2) - beacons(i,2)) / geo_dist;
    end
    
    % WEIGHTED Least Squares: dx = inv(H'*W*H) * H'*W * dP
    % This prioritizes the beacons with stronger signals
    lhs = H' * W * H;
    rhs = H' * W * dP;
    err_correction = lhs \ rhs;
    
    indoor_est = indoor_est + err_correction';
end

plot(indoor_est(1), indoor_est(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Weighted Est');
legend('Location', 'southoutside');

%% Print Comparison
fprintf('--- SYSTEM COMPARISON ---\n');
% fprintf('GNSS Error (Time-based, 3D):  %.2f m\n', norm(gnss_est(1:3) - gnss_true_pos));
fprintf('Indoor Error (RSSI, 2D):      %.2f m\n', norm(indoor_est - indoor_true_pos));
fprintf('Note how Indoor circles do not intersect perfectly due to RSSI noise.\n');