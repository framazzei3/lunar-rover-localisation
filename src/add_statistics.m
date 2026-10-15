function est = add_statistics(est, truth)
%ADD_STATISTICS Position error and covariance size of an estimate.
%   pos_error  distance between estimated and true position [km]
%   sqrt_detP  square root of the determinant of the 3x3 covariance
%   sigma_pos  1 sigma semi major axis of the position covariance [km]

N = size(est.q, 2);
est.pos_error = hypot(est.q(1,:).' - truth.x, est.q(2,:).' - truth.y);
est.sqrt_detP = zeros(N, 1);
est.sigma_pos = zeros(N, 1);
for k = 1:N
    est.sqrt_detP(k) = sqrt(det(est.P(:,:,k)));
    est.sigma_pos(k) = sqrt(max(eig(est.P(1:2,1:2,k))));
end
end
