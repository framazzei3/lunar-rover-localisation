%MAIN Autonomous localisation of a lunar rover.
%   Task 1: plan a safe path to two targets and drive it with a moving
%           target follower and a move to a point controller.
%   Task 2: reconstruct the path with the odometer only (dead reckoning).
%   Task 3: reconstruct it with odometer and lidar in a Kalman filter,
%           using known landmarks (by default a second order,
%           observability constrained EKF with measurement underweighting;
%           see src/ekf_localisation.m and monte_carlo.m).
%
%   Run from this folder. Figures go to results/figures and a summary of
%   the numbers to results/summary.txt. Needs base MATLAB only.

clear; close all; clc
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));

p = rover_parameters();
rng(p.seed, 'twister');

%% Task 1: path planning and control
truth = simulate_drive(p);

%% Task 2 and 3: localisation, with the same odometer readings
odo = simulate_odometry(truth, p);
dr  = dead_reckoning(odo, truth, p);
ekf = ekf_localisation(odo, truth, p);

%% Results
plot_results(p, truth, dr, ekf, fullfile(root, 'results', 'figures'));

% Clearance from the ridges of the path and of its ellipses
ellipse_clearance = @(est) min(arrayfun(@(k) ridge_clearance( ...
    error_ellipse(est.q(1:2,k), est.P(1:2,1:2,k), p.confidence), p.ridges), 1:size(est.q,2)));

lines = {
    sprintf('Task 1')
    sprintf('  Arrival at target 1           %8.2f h', truth.t_arrival(1))
    sprintf('  Departure from target 1       %8.2f h', truth.t_departure(2))
    sprintf('  Arrival at target 2           %8.2f h', truth.t_arrival(2))
    sprintf('  Path length                   %8.2f km', sum(hypot(diff(truth.x), diff(truth.y))))
    sprintf('  Ridge clearance, driven path  %8.2f km', ridge_clearance([truth.x.'; truth.y.'], p.ridges))
    sprintf('Task 2, dead reckoning')
    sprintf('  Final position error          %8.3f km', dr.pos_error(end))
    sprintf('  Maximum position error        %8.3f km', max(dr.pos_error))
    sprintf('  Final 1 sigma (major axis)    %8.3f km', dr.sigma_pos(end))
    sprintf('  Final sqrt(det P)             %8.2e', dr.sqrt_detP(end))
    sprintf('  Ridge clearance, %2.0f%% ellipses %8.3f km', 100*p.confidence, ellipse_clearance(dr))
    sprintf('Task 3, filter ''%s'', underweighting %g', p.filter, p.underweight)
    sprintf('  Lidar updates                 %8d of %d samples', nnz(ekf.updated), numel(truth.t) - 1)
    sprintf('  Final position error          %8.1f m', 1e3*ekf.pos_error(end))
    sprintf('  Maximum position error        %8.1f m', 1e3*max(ekf.pos_error))
    sprintf('  Maximum 1 sigma (major axis)  %8.1f m', 1e3*max(ekf.sigma_pos))
    sprintf('  Final sqrt(det P)             %8.2e', ekf.sqrt_detP(end))
    sprintf('  Ridge clearance, %2.0f%% ellipses %8.3f km', 100*p.confidence, ellipse_clearance(ekf))
    };
summary = strjoin(lines, newline);
disp(summary)

fid = fopen(fullfile(root, 'results', 'summary.txt'), 'w');
fprintf(fid, '%s\n', summary);
fclose(fid);
