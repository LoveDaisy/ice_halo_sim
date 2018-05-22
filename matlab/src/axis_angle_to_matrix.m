function matR = axis_angle_to_matrix(u, theta)
% This function returns the rotation matrix representing
% the rotation defined by an axis u and an angle theta.
% See https://www.wikiwand.com/en/Rotation_matrix#/Conversion_from_and_to_axis%E2%80%93angle
% INPUT
%   u:      the rotation axis.
%   theta:  the rotation angle.
% OUTPUT
%   matR:   the ratation matrix.

assert(length(size(u)) == 2 && max(size(u)) == 3 && min(size(u)) == 1);
assert(length(theta) == 1);

c = cosd(theta); s = sind(theta);
matR = [c + u(1)^2*(1-c), u(1)*u(2)*(1-c) - u(3)*s, u(1)*u(3)*(1-c) + u(2)*s;
        u(2)*u(1)*(1-c) + u(3)*s, c + u(2)^2*(1-c), u(2)*u(3)*(1-c) - u(1)*s;
        u(3)*u(1)*(1-c) - u(2)*s, u(3)*u(2)*(1-c) + u(1)*s, c + u(3)^2*(1-c)];
end