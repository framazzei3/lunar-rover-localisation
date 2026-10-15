function d = ridge_clearance(xy, ridges)
%RIDGE_CLEARANCE Smallest distance from a set of points to the ridges [km].
%   xy is 2 x N; ridges has one horizontal segment per row,
%   [x_start, x_end, y]. A result of zero means a point lies on a ridge.

d = inf;
for i = 1:size(ridges, 1)
    x1 = min(ridges(i,1:2));
    x2 = max(ridges(i,1:2));
    dx = max(max(x1 - xy(1,:), 0), xy(1,:) - x2);
    dy = xy(2,:) - ridges(i,3);
    d  = min(d, min(hypot(dx, dy)));
end
end
