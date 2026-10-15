function odo = simulate_odometry(truth, p)
%SIMULATE_ODOMETRY Noisy odometer readings along the true path.
%   Between consecutive samples the odometer measures the travelled
%   distance and the change of heading, each with Gaussian white noise.
%   While the rover is parked the wheels do not turn, so the readings are
%   exactly zero.

dd  = hypot(diff(truth.x), diff(truth.y));
dth = wrap_to_pi(diff(truth.theta));

odo.moving = dd > 1e-12;
n = numel(dd);
odo.dd  = dd  + p.sigma_d  * randn(n, 1) .* odo.moving;
odo.dth = dth + p.sigma_ha * randn(n, 1) .* odo.moving;
end
