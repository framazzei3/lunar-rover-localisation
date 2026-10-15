function [q, P, Q] = odometry_step(q, P, dd, dth, moving, p)
%ODOMETRY_STEP Propagate pose and covariance with one odometer reading.
%   q = [x; y; theta]. Over one sample the rover moves along an arc of
%   constant curvature, whose chord points along the mean heading
%   m = theta + dth/2:
%       x' = x + dd cos(m),  y' = y + dd sin(m),  theta' = theta + dth
%   Using the initial heading instead would add a systematic error at
%   every turn, which the filter does not know about.
%
%   P is propagated as P' = Fq P Fq' + Q, with Fq the Jacobian with respect
%   to the state and Q = Fn V Fn' the odometer noise mapped through the
%   Jacobian Fn with respect to dd and dth. Q is returned so that a filter
%   can rebuild P' with a different Fq (see ekf_localisation, 'oc').

m = q(3) + dth/2;
Fq = [1, 0, -dd*sin(m)
      0, 1,  dd*cos(m)
      0, 0,  1];
Fn = [cos(m), -dd/2*sin(m)
      sin(m),  dd/2*cos(m)
      0,       1];
V = diag([p.sigma_d, p.sigma_ha].^2) * moving;
Q = Fn*V*Fn.';

q = [q(1) + dd*cos(m); q(2) + dd*sin(m); wrap_to_pi(q(3) + dth)];
P = Fq*P*Fq.' + Q;
end
