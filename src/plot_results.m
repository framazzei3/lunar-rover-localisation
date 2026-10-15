function plot_results(p, truth, dr, ekf, out_dir)
%PLOT_RESULTS Figures for the three tasks, saved as PNG in out_dir.

if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
t = truth.t;

%% Task 1: planned and driven trajectory
f = map_figure(p);
ref_x = vertcat(truth.ref.x);
ref_y = vertcat(truth.ref.y);
h(1) = plot(ref_x, ref_y, 'w-', 'LineWidth', 2);
h(2) = plot(truth.x, truth.y, 'r--', 'LineWidth', 2);
via = [p.via1, p.via2];
h(3) = plot(via(1,:), via(2,:), 'yo', 'MarkerSize', 9, 'LineWidth', 1.5);
hm = plot_markers(p);
map_legend([h, hm], {'Reference trajectory', 'Driven trajectory', 'Via points', ...
    'Start', 'Target 1', 'Target 2', 'Ridges'});
save_figure(f, fullfile(out_dir, 'task1_trajectory.png'));

%% Task 1: speed and steering angle
f = figure('Color', 'w', 'Position', [100 100 1200 420]);
subplot(1, 2, 1)
plot(t, truth.v, 'LineWidth', 2); grid on
xlabel('Time [h]'); ylabel('Speed [km/h]')
ylim([0, 1.1*p.v_max])
subplot(1, 2, 2)
plot(t, rad2deg(truth.gamma), 'LineWidth', 2); grid on
xlabel('Time [h]'); ylabel('Steering angle [deg]')
save_figure(f, fullfile(out_dir, 'task1_speed_steering.png'));

%% Task 2 and 3: estimated trajectories on the map
plot_estimate(p, truth, dr,  10, 'Dead reckoning',         fullfile(out_dir, 'task2_dead_reckoning.png'));
plot_estimate(p, truth, ekf,  2, 'Kalman filter', fullfile(out_dir, 'task3_ekf.png'));

%% Square root of the covariance determinant
f = figure('Color', 'w', 'Position', [100 100 1200 420]);
subplot(1, 2, 1)
plot(t, dr.sqrt_detP, 'LineWidth', 2); grid on
xlabel('Time [h]'); ylabel('$\sqrt{\det P}$', 'Interpreter', 'latex')
title('Dead reckoning')
subplot(1, 2, 2)
plot(t, ekf.sqrt_detP, 'LineWidth', 2); grid on
xlabel('Time [h]'); ylabel('$\sqrt{\det P}$', 'Interpreter', 'latex')
title('Kalman filter')
save_figure(f, fullfile(out_dir, 'sqrt_det_P.png'));

%% Position error against the 1 sigma uncertainty
f = figure('Color', 'w', 'Position', [100 100 900 480]);
semilogy(t, 1e3*dr.pos_error,  'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); hold on
semilogy(t, 1e3*dr.sigma_pos,  '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
semilogy(t, 1e3*ekf.pos_error, 'Color', [0 0.45 0.74], 'LineWidth', 1.5);
semilogy(t, 1e3*ekf.sigma_pos, '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.5);
grid on
xlabel('Time [h]'); ylabel('Position [m]')
legend('Dead reckoning error', 'Dead reckoning 1\sigma', 'EKF error', 'EKF 1\sigma', 'Location', 'southeast')
save_figure(f, fullfile(out_dir, 'position_error.png'));
end

function plot_estimate(p, truth, est, every, label, file)
f = map_figure(p);
h(1) = plot(truth.x, truth.y, 'r-', 'LineWidth', 1.5);
h(2) = plot(est.q(1,:), est.q(2,:), 'w-', 'LineWidth', 1.5);
for k = 1:every:size(est.q, 2)
    xy = error_ellipse(est.q(1:2,k), est.P(1:2,1:2,k), p.confidence);
    h(3) = plot(xy(1,:), xy(2,:), 'c-', 'LineWidth', 0.5);
end
h(4) = plot(p.landmarks(1,:), p.landmarks(2,:), 'yo', 'MarkerSize', 9, 'LineWidth', 1.5);
hm = plot_markers(p);
map_legend([h, hm], {'True trajectory', label, ...
    sprintf('%g%% ellipses', 100*p.confidence), 'Landmarks', ...
    'Start', 'Target 1', 'Target 2', 'Ridges'});
save_figure(f, file);
end

function f = map_figure(p)
f = figure('Color', 'w', 'Position', [100 100 640 960]);
imshow(imread(p.map_file), 'XData', p.map_x, 'YData', p.map_y);
hold on
axis on
axis([p.map_x, p.map_y])
xlabel('X [km]'); ylabel('Y [km]')
end

function h = plot_markers(p)
h(1) = plot(p.start(1),   p.start(2),   'r*', 'MarkerSize', 10, 'LineWidth', 1.5);
h(2) = plot(p.target1(1), p.target1(2), 'c*', 'MarkerSize', 10, 'LineWidth', 1.5);
h(3) = plot(p.target2(1), p.target2(2), 'g*', 'MarkerSize', 10, 'LineWidth', 1.5);
for i = 1:size(p.ridges, 1)
    h(4) = plot(p.ridges(i,1:2), p.ridges(i,[3 3]), 'k-', 'LineWidth', 4);
end
end

function map_legend(h, labels)
legend(h, labels, 'TextColor', 'w', 'EdgeColor', 'w', 'Color', 'none', 'Location', 'northwest');
end

function save_figure(f, file)
if isprop(f, 'Theme')
    f.Theme = 'light';      % newer MATLAB releases may default to dark
end
exportgraphics(f, file, 'Resolution', 200);
close(f);
end
