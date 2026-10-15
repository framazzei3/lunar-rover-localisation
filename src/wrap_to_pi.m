function a = wrap_to_pi(a)
%WRAP_TO_PI Wrap angles to the interval [-pi, pi).
%   Same as wrapToPi, without needing the Mapping Toolbox.
a = mod(a + pi, 2*pi) - pi;
end
