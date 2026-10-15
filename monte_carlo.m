%MONTE_CARLO Consistency of the three filters over many noise realisations.
%   The driven path is the same in every run; odometer and lidar noise
%   change. For each filter it computes the normalised estimation error
%   squared (NEES) of the pose,
%       NEES = e' P^-1 e,   e = estimated pose - true pose,
%   averaged over the runs. A consistent filter has a mean NEES equal to
%   the state dimension (3) and stays within the chi square bounds; a
%   larger value means the filter is overconfident.
%
%   Results go to results/figures/monte_carlo_nees.png and
%   results/monte_carlo.txt.

clear; close all; clc
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'));

n_runs  = 50;
% Filter, underweighting factor and label of each configuration compared
configs = {'ekf',  0, 'EKF'
           'iekf', 0, 'Iterated EKF'
           'oc',   0, 'OC-EKF'
           'oc2',  0, 'Second order OC-EKF'
           'oc2',  1, 'Second order OC-EKF, underweighting'};
methods = configs(:,1).';
betas   = cell2mat(configs(:,2)).';
names   = configs(:,3).';

p = rover_parameters();
rng(p.seed, 'twister');
truth = simulate_drive(p);
N = numel(truth.t);

nees    = zeros(N, n_runs, numel(methods));
pos_err = zeros(N, n_runs, numel(methods));
inside  = zeros(N, n_runs, numel(methods));
k95 = -2*log(1 - p.confidence);          % chi square 95% quantile, 2 dof

for run = 1:n_runs
    rng(p.seed + run, 'twister');
    odo = simulate_odometry(truth, p);
    s = rng;                             % same lidar noise for every filter
    for m = 1:numel(methods)
        rng(s);
        pm = p;
        pm.underweight = betas(m);
        est = ekf_localisation(odo, truth, pm, methods{m});
        e = est.q - [truth.x.'; truth.y.'; truth.theta.'];
        e(3,:) = wrap_to_pi(e(3,:));
        for k = 1:N
            nees(k, run, m) = e(:,k).' * (est.P(:,:,k) \ e(:,k));
            inside(k, run, m) = e(1:2,k).' * (est.P(1:2,1:2,k) \ e(1:2,k)) <= k95;
        end
        pos_err(:, run, m) = est.pos_error;
    end
end

% Two sided 95% bounds of the mean NEES over n_runs (chi square with
% 3 n_runs dof, Wilson-Hilferty approximation, no toolbox needed)
dof = 3*n_runs;
wh  = @(z) dof*(1 - 2/(9*dof) + z*sqrt(2/(9*dof)))^3 / n_runs;
bounds = [wh(-1.96), wh(1.96)];

%% Figure
f = figure('Color', 'w', 'Position', [100 100 1000 480]);
colors = [0.85 0.33 0.10; 0.47 0.67 0.19; 0.49 0.18 0.56; 0.93 0.69 0.13; 0 0.45 0.74];
for m = 1:numel(methods)
    semilogy(truth.t, mean(nees(:,:,m), 2), 'Color', colors(m,:), 'LineWidth', 1.5); hold on
end
yline(3, 'k-', 'LineWidth', 1);
yline(bounds(1), 'k--');
yline(bounds(2), 'k--');
grid on
xlabel('Time [h]'); ylabel('Mean pose NEES')
legend([names, {'Expected (3)', '95% bounds'}], 'Location', 'northwest')
if isprop(f, 'Theme'), f.Theme = 'light'; end
exportgraphics(f, fullfile(root, 'results', 'figures', 'monte_carlo_nees.png'), 'Resolution', 200);
close(f)

%% Summary
moving = [false; true(N-1, 1)];          % skip the initial sample
lines = {sprintf('Monte Carlo, %d runs, %d samples each', n_runs, N)
         sprintf('Mean NEES should be 3, with 95%% bounds [%.2f, %.2f]', bounds)
         ''
         sprintf('%-37s %10s %12s %12s %12s %11s', 'Filter', 'Mean NEES', 'Median NEES', 'Inside 95%', 'RMS error', 'Max error')};
for m = 1:numel(methods)
    x = nees(moving,:,m);
    in = mean(inside(moving,:,m), 'all');
    rms_err = sqrt(mean(pos_err(moving,:,m).^2, 'all'));
    lines{end+1} = sprintf('%-37s %10.1f %12.2f %11.1f%% %10.1f m %9.1f m', names{m}, mean(x(:)), ...
        median(x(:)), 100*in, 1e3*rms_err, 1e3*max(pos_err(:,:,m), [], 'all')); %#ok<SAGROW>
end
summary = strjoin(lines, newline);
disp(summary)
fid = fopen(fullfile(root, 'results', 'monte_carlo.txt'), 'w');
fprintf(fid, '%s\n', summary);
fclose(fid);
