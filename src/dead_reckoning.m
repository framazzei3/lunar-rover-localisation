function est = dead_reckoning(odo, truth, p)
%DEAD_RECKONING Task 2: localisation with the odometer only.
%   Starts from the true initial pose with covariance P0 and propagates
%   pose and covariance with every odometer reading.

N = numel(truth.t);
est.q = zeros(3, N);
est.P = zeros(3, 3, N);
est.q(:,1)   = p.start;
est.P(:,:,1) = p.P0;

for k = 1:N-1
    [est.q(:,k+1), est.P(:,:,k+1)] = odometry_step(est.q(:,k), est.P(:,:,k), ...
        odo.dd(k), odo.dth(k), odo.moving(k), p);
end

est = add_statistics(est, truth);
end
