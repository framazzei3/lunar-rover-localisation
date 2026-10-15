function xy = error_ellipse(mu, P, confidence, n_points)
%ERROR_ELLIPSE Points on the uncertainty ellipse of a 2D Gaussian.
%   xy = ERROR_ELLIPSE(mu, P, confidence) returns a 2 x n_points array on
%   the ellipse that contains the given probability, for mean mu and 2x2
%   covariance P. For two degrees of freedom the chi square quantile has a
%   closed form, so no toolbox is needed:
%       k^2 = chi2inv(confidence, 2) = -2 log(1 - confidence)
%   and the semi axes are k times the square roots of the eigenvalues.

if nargin < 4
    n_points = 100;
end
k = sqrt(-2*log(1 - confidence));
[V, D] = eig((P + P.')/2);
a = linspace(0, 2*pi, n_points);
xy = mu(:) + k * V * sqrt(max(D, 0)) * [cos(a); sin(a)];
end
