function [Rs, Rp] = calculate_reflect_ratio(inc_angle, n1, n2)
% This function returns the reflective ratio using Fresnel equations
% See https://www.wikiwand.com/en/Fresnel_equations
% INPUT
%   inc_angle:  incident angle, in degree.
%   n1:         refractive index before.
%   n2:         refractive index after.
% OUTPUT
%   Rs:         reflective ratio of s-polarized light.
%   Rp:         reflective ratio of p-polarized light.

assert(all(size(n1) == size(n2)));
assert(size(inc_angle, 1) == size(n1, 1) || length(n1) == 1);
assert(size(inc_angle, 1) == length(inc_angle));

c = cosd(inc_angle);
s = sind(inc_angle);

d = max(1 - ((n1 ./ n2) .* s).^2, 0);

Rs = ((n1 .* c - n2 .* sqrt(d)) ./ (n1 .* c + n2 .* sqrt(d))).^2;
Rp = ((n1 .* sqrt(d) - n2 .* c) ./ (n1 .* sqrt(d) + n2 .* c)).^2;
end