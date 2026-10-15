function ref = plan_leg(p, start_xy, via, target, t0)
%PLAN_LEG Reference trajectory for one leg, from start_xy to target.
%   The reference starts lead_distance ahead of the rover, so that the
%   moving target follower has the right spacing from the first instant,
%   passes through the via points and stops at the target. Each segment is
%   travelled at speed_fraction of the maximum speed.

first = start_xy(:) + [p.lead_distance; 0];
pts   = [first, via, target(:)];

seg_length = sqrt(sum(diff(pts, 1, 2).^2, 1));
td = seg_length(:) / (p.speed_fraction * p.v_max);   % time per segment [h]
tk = p.blend_time * ones(numel(td) + 1, 1);

[t, x] = lspb_via_points(first(1), target(1), via(1,:), tk, td, p.ref_dt);
[~, y] = lspb_via_points(first(2), target(2), via(2,:), tk, td, p.ref_dt);

% Remove repeated time samples at the junctions between segments
[ref.t, keep] = unique(t(:) + t0);
ref.x = x(keep).';
ref.y = y(keep).';
ref.x = ref.x(:);
ref.y = ref.y(:);
end
