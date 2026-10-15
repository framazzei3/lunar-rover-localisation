function truth = simulate_drive(p)
%SIMULATE_DRIVE Task 1: plan and drive the two legs to the targets.
%   Integrates the kinematic bicycle model under rover_controller, first
%   from the start to target 1, then, after dwell_time at target 1, to
%   target 2. The result is sampled every odo_dt and is the true path used
%   by the localisation tasks.

% Leg 1
ref1 = plan_leg(p, p.start(1:2), p.via1, p.target1, 0);
sol1 = drive_leg(p, ref1, p.target1, 0, [p.start; 0]);
t_arr1 = sol1.x(end);

% Leg 2, planned from where the rover actually stopped
t_dep2 = t_arr1 + p.dwell_time;
X_stop = sol1.y(:, end);
ref2 = plan_leg(p, X_stop(1:2), p.via2, p.target2, t_dep2);
sol2 = drive_leg(p, ref2, p.target2, t_dep2, [X_stop(1:3); 0]);
t_arr2 = sol2.x(end);

% Sample the whole drive at the odometry rate
t = (0:p.odo_dt:t_arr2).';
if t(end) < t_arr2
    t(end+1) = t_arr2;
end
X = zeros(numel(t), 4);
v = zeros(numel(t), 1);
gamma = zeros(numel(t), 1);
for k = 1:numel(t)
    if t(k) <= t_arr1
        X(k,:) = deval(sol1, t(k)).';
        [v(k), gamma(k)] = rover_controller(t(k), X(k,:).', ref1, p.target1, p);
    elseif t(k) < t_dep2
        X(k,:) = X_stop.';              % parked at target 1
    else
        X(k,:) = deval(sol2, t(k)).';
        [v(k), gamma(k)] = rover_controller(t(k), X(k,:).', ref2, p.target2, p);
    end
end

truth.t      = t;
truth.x      = X(:,1);
truth.y      = X(:,2);
truth.theta  = X(:,3);
truth.v      = v;
truth.gamma  = gamma;
truth.ref    = [ref1, ref2];
truth.t_arrival   = [t_arr1, t_arr2];
truth.t_departure = [0, t_dep2];
end

function sol = drive_leg(p, ref, target, t0, X0)
% Integrate one leg until the rover is within arrival_tol of the target
dyn = @(t, X) rover_dynamics(t, X, ref, target, p);
evt = @(t, X) arrival_event(X, target, p);
opts = odeset('Events', evt, 'RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', 0.01);
sol = ode45(dyn, [t0, ref.t(end) + p.max_extra_time], X0, opts);
if isempty(sol.xe)
    warning('simulate_drive:notReached', ...
        'Target (%.2f, %.2f) not reached within the allowed time.', target(1), target(2));
end
end

function dX = rover_dynamics(t, X, ref, target, p)
% Kinematic bicycle model, plus the integral of the following error
[v, gamma, de_int] = rover_controller(t, X, ref, target, p);
theta = X(3);
dX = [v*cos(theta); v*sin(theta); v/p.L*tan(gamma); de_int];
end

function [value, isterminal, direction] = arrival_event(X, target, p)
value      = hypot(X(1) - target(1), X(2) - target(2)) - p.arrival_tol;
isterminal = 1;
direction  = -1;
end
