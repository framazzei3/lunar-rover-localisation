function est = ekf_localisation(odo, truth, p, method)
%EKF_LOCALISATION Task 3: Kalman filtering of odometer and lidar.
%   est = EKF_LOCALISATION(odo, truth, p, method) predicts with the same
%   odometer readings as dead reckoning and updates with the range and
%   bearing to the closest landmark, whenever one is within lidar range.
%
%   method (default p.filter)
%     'ekf'   standard extended Kalman filter
%     'iekf'  iterated EKF: the measurement model is linearised again
%             around the corrected pose until it converges
%     'oc'    observability constrained EKF
%     'ekf2'  EKF with second order measurement terms
%     'oc2'   observability constrained EKF with second order terms
%   p.underweight > 0 adds measurement underweighting to any of them.
%
%   Two effects make the standard EKF overconfident here, and each option
%   addresses one of them.
%
%   Why 'oc'. The lidar never sees more than one landmark at a time. With
%   a single landmark, rotating the whole pose (position and heading)
%   about it leaves range and bearing unchanged, and the odometer only
%   measures relative motion, so that rotation is unobservable. Its
%   direction in the state space is
%       N = [J (p - l); 1],   J = [0 -1; 1 0],
%   with p the rover position and l the landmark. The standard EKF
%   evaluates the propagation Jacobian at the updated pose and the
%   measurement Jacobian at the predicted pose; the two do not agree on N,
%   and the filter gains information along a direction where there is
%   none, becoming overconfident (Huang, Mourikis and Roumeliotis, 2010).
%   The observability constrained EKF builds the propagation Jacobian
%   from the predicted poses,
%       Fq = [I, J (p_pred(k+1) - p_pred(k)); 0 0 1],
%   so that Fq N(k) = N(k+1), and evaluates the measurement Jacobian at
%   the predicted pose, where H N = 0 holds exactly for range and bearing.
%   The estimate itself is propagated as in the EKF; only the covariance
%   changes. The update is not iterated, which would break H N = 0.
%
%   Why second order terms and underweighting. After a stretch without
%   landmarks the position is uncertain by hundreds of metres, while the
%   lidar range is accurate to 2 cm. Over that spread the range is far
%   from linear (about e^2/2r, metres at 2 km), so the first measurement
%   would be trusted far beyond what the linear model justifies. The
%   second order filter (Bar-Shalom; Gelb) adds the curvature of the
%   measurement model to the predicted measurement and to its covariance;
%   underweighting (Lear, for the Space Shuttle; Zanetti, 2012) further
%   scales H P H' in the gain by (1 + underweight).
%
%   The covariance update uses the Joseph form, which keeps P symmetric
%   and positive definite.

if nargin < 4
    method = p.filter;
end
n_iter = 1;
if strcmp(method, 'iekf')
    n_iter = 10;
end
constrained  = any(strcmp(method, {'oc', 'oc2'}));
second_order = any(strcmp(method, {'ekf2', 'oc2'}));
N = numel(truth.t);
W = diag([p.sigma_r, p.sigma_b].^2);
J = [0 -1; 1 0];

est.method   = method;
est.q        = zeros(3, N);
est.P        = zeros(3, 3, N);
est.q(:,1)   = p.start;
est.P(:,:,1) = p.P0;
est.updated  = false(N, 1);
q_pred_prev  = p.start;          % predicted pose at the previous sample

for k = 1:N-1
    % Prediction
    [q_pred, P, Q] = odometry_step(est.q(:,k), est.P(:,:,k), odo.dd(k), odo.dth(k), odo.moving(k), p);
    if constrained
        Fq = [eye(2), J*(q_pred(1:2) - q_pred_prev(1:2)); 0 0 1];
        P  = Fq*est.P(:,:,k)*Fq.' + Q;
    end
    q_pred_prev = q_pred;
    q = q_pred;

    % Update, if a landmark is in view from the true pose
    q_true = [truth.x(k+1); truth.y(k+1); truth.theta(k+1)];
    [z, lm] = lidar_measurement(q_true, p);
    if ~isempty(z)
        for i = 1:n_iter
            [h, H, Hr, Hb] = lidar_model(q, lm);
            S = H*P*H.' + W;
            if second_order
                % Second order terms: mean shift and extra covariance from
                % the curvature of the measurement model
                h = h + 0.5*[trace(Hr*P); trace(Hb*P)];
                S = S + 0.5*[trace(Hr*P*Hr*P), trace(Hr*P*Hb*P)
                             trace(Hb*P*Hr*P), trace(Hb*P*Hb*P)];
            end
            R_eff = S - H*P*H.';      % lidar noise plus second order terms
            if p.underweight > 0
                % Measurement underweighting (Lear; Zanetti 2012): smaller
                % gain while the prior uncertainty dominates the noise
                S = S + p.underweight*(H*P*H.');
            end
            K = P*H.' / S;
            innovation = z - h - H*(q_pred - q);
            innovation(2) = wrap_to_pi(innovation(2));
            q_new = q_pred + K*innovation;
            q_new(3) = wrap_to_pi(q_new(3));
            converged = norm(q_new(1:2) - q(1:2)) < 1e-9;
            q = q_new;
            if converged
                break
            end
        end
        if n_iter > 1
            [~, H] = lidar_model(q, lm);
            R_eff = W;
            K = P*H.' / (H*P*H.' + W);
        end
        I_KH = eye(3) - K*H;
        P = I_KH*P*I_KH.' + K*R_eff*K.';
        est.updated(k+1) = true;
    end

    est.q(:,k+1)   = q;
    est.P(:,:,k+1) = P;
end

est = add_statistics(est, truth);
end

function [h, H, Hr, Hb] = lidar_model(q, lm)
% Predicted range and bearing to landmark lm from pose q, their Jacobian H
% and the Hessians of range (Hr) and bearing (Hb) with respect to the pose
dx = lm(1) - q(1);
dy = lm(2) - q(2);
r2 = dx^2 + dy^2;
r  = sqrt(r2);
h  = [r; atan2(dy, dx) - q(3)];
H  = [-dx/r,  -dy/r,   0
       dy/r2, -dx/r2, -1];
Hr = zeros(3);
Hr(1:2,1:2) = (eye(2) - [dx; dy]*[dx, dy]/r2) / r;
Hb = zeros(3);
Hb(1:2,1:2) = [2*dx*dy,     dy^2 - dx^2
               dy^2 - dx^2, -2*dx*dy] / r2^2;
end
