function [t, q, qd, qdd] = lspb_via_points(q0, qf, q_via, tk, td, dt)
%LSPB_VIA_POINTS Linear segments with parabolic blends through via points.
%   [t, q, qd, qdd] = LSPB_VIA_POINTS(q0, qf, q_via, tk, td, dt) builds a
%   one dimensional trajectory from q0 to qf through the via points q_via.
%
%   q0, qf  start and end value
%   q_via   row vector of via points
%   tk      blend duration at each path point (start, via points, end)
%   td      time between consecutive path points
%   dt      sampling step
%
%   Returns time, position, velocity and acceleration, sampled every dt.
%   The first and last segments end with a blend that starts or stops at
%   rest, as in Craig, Introduction to Robotics, chapter 7.

q  = [q0, q_via(:).', qf];   % all path points
tk = tk(:);
td = td(:);
n  = numel(tk);              % number of path points

% Velocities on the linear segments and accelerations in the blends
v = zeros(n - 1, 1);
a = zeros(n, 1);

v(1) = (q(2) - q(1)) / (td(1) - tk(1)/2);
a(1) = v(1) / tk(1);
for k = 2:n-2
    v(k) = (q(k+1) - q(k)) / td(k);
end
v(n-1) = (q(n) - q(n-1)) / (td(n-1) - tk(n)/2);
for k = 2:n-1
    a(k) = (v(k) - v(k-1)) / tk(k);
end
a(n) = -v(n-1) / tk(n);

% First blend, starting at rest
tb  = 0:dt:tk(1);
t   = tb;
q_t = q(1) + a(1)/2 * tb.^2;
qd  = a(1) * tb;
qdd = a(1) * ones(size(tb));

% First linear segment
tl = 0:dt:(td(1) - tk(1) - tk(2)/2);
[t, q_t, qd, qdd] = append_linear(t, q_t, qd, qdd, tl, v(1));

% Intermediate blends and linear segments
for k = 2:n-2
    tb = 0:dt:tk(k);
    [t, q_t, qd, qdd] = append_blend(t, q_t, qd, qdd, tb, a(k));
    tl = 0:dt:(td(k) - tk(k)/2 - tk(k+1)/2);
    [t, q_t, qd, qdd] = append_linear(t, q_t, qd, qdd, tl, v(k));
end

% Last intermediate blend, last linear segment and final blend to rest
tb = 0:dt:tk(n-1);
[t, q_t, qd, qdd] = append_blend(t, q_t, qd, qdd, tb, a(n-1));
tl = 0:dt:(td(n-1) - tk(n) - tk(n-1)/2);
[t, q_t, qd, qdd] = append_linear(t, q_t, qd, qdd, tl, v(n-1));
tb = 0:dt:tk(n);
[t, q_t, qd, qdd] = append_blend(t, q_t, qd, qdd, tb, a(n));

q = q_t;
end

function [t, q, qd, qdd] = append_linear(t, q, qd, qdd, tl, v)
q   = [q,   q(end) + v*tl];
qd  = [qd,  v*ones(size(tl))];
qdd = [qdd, zeros(size(tl))];
t   = [t,   t(end) + tl];
end

function [t, q, qd, qdd] = append_blend(t, q, qd, qdd, tb, a)
q   = [q,   q(end) + qd(end)*tb + a/2*tb.^2];
qd  = [qd,  qd(end) + a*tb];
qdd = [qdd, a*ones(size(tb))];
t   = [t,   t(end) + tb];
end
