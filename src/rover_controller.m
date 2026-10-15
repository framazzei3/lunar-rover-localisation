function [v, gamma, de_int] = rover_controller(t, X, ref, target, p)
%ROVER_CONTROLLER Speed and steering commands for the rover.
%   Far from the target the rover follows a moving target: a PI law on
%   speed keeps it lead_distance behind the reference, a proportional law
%   on steering points it at the reference. Within approach_radius of the
%   target it switches to move to a point, and stops within arrival_tol.
%
%   X = [x; y; theta; e_int], where e_int is the integral of the following
%   error. de_int is its derivative, frozen while the speed is saturated
%   (anti windup).

x = X(1);  y = X(2);  theta = X(3);  e_int = X(4);

d_target = hypot(target(1) - x, target(2) - y);

if d_target < p.approach_radius
    % Move to a point
    e = 0;
    heading_cmd = atan2(target(2) - y, target(1) - x);
    if d_target > p.arrival_tol
        v = p.Kv * d_target;
    else
        v = 0;
    end
else
    % Follow the moving target; after the reference ends, hold its last point
    tq   = min(t, ref.t(end));
    xdes = interp1(ref.t, ref.x, tq);
    ydes = interp1(ref.t, ref.y, tq);
    e = hypot(xdes - x, ydes - y) - p.lead_distance;
    heading_cmd = atan2(ydes - y, xdes - x);
    v = p.Kv * e + p.Ki * e_int;
end

% Forward motion only, up to the maximum speed
v_sat = min(max(v, 0), p.v_max);
if v_sat ~= v && sign(e) == sign(v - v_sat)
    de_int = 0;     % saturated and the error would push further: stop integrating
else
    de_int = e;
end
v = v_sat;

gamma = p.Kh * wrap_to_pi(heading_cmd - theta);
end
