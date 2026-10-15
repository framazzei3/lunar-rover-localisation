function [z, landmark] = lidar_measurement(q_true, p)
%LIDAR_MEASUREMENT Range and bearing to the closest landmark in view.
%   Returns z = [range; bearing] with Gaussian noise, and the landmark
%   position, if the closest landmark is within lidar_range; otherwise z
%   and landmark are empty. The bearing is measured from the rover heading.

dx = p.landmarks(1,:) - q_true(1);
dy = p.landmarks(2,:) - q_true(2);
[r, i] = min(hypot(dx, dy));

if r > p.lidar_range
    z = [];
    landmark = [];
    return
end

landmark = p.landmarks(:, i);
bearing  = atan2(dy(i), dx(i)) - q_true(3);
z = [r + p.sigma_r*randn; wrap_to_pi(bearing + p.sigma_b*randn)];
end
