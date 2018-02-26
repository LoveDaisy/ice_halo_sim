function [p, t, alpha, beta] = intersect_line_face(pts0, d0, face_base, face_point)
% This function calculates the intersect point of a line and a plane face by solving
% following equations:
%
%   p = pts0 + t * d0
%   p = alpha * face_base(1,:) + beta * face_base(2,:) + face_point
%
% INPUT:
%   pts0:       origin point. n * 3 array.
%   d0:         direction. n * 3 array.
%   face_base:  base vector of the face. 2 * 3 array.
%   face_point: origin point of the face. 1 * 3 array.
% OUTPUT
%   p:          the intersection point. n * 3 array.
%   t:          n * 1 array.
%   alpha:      n * 1 array.
%   beta:       n * 1 array.

a = face_base(1,1)*face_base(2,2)*face_point(3) + face_base(1,2)*face_base(2,3)*face_point(1) + ...
    face_base(1,3)*face_base(2,1)*face_point(2) - face_base(1,3)*face_base(2,2)*face_point(1) - ...
    face_base(1,2)*face_base(2,1)*face_point(3) - face_base(1,1)*face_base(2,3)*face_point(2);
b = pts0(:,1)*face_base(1,2)*face_base(2,3) + pts0(:,2)*face_base(1,3)*face_base(2,1) + ...
    pts0(:,3)*face_base(1,1)*face_base(2,2) - pts0(:,1)*face_base(1,3)*face_base(2,2) - ...
    pts0(:,2)*face_base(1,1)*face_base(2,3) - pts0(:,3)*face_base(1,2)*face_base(2,1);
c = d0(:,1)*face_base(1,2)*face_base(2,3) + d0(:,2)*face_base(1,3)*face_base(2,1) + ...
    d0(:,3)*face_base(1,1)*face_base(2,2) - d0(:,1)*face_base(1,3)*face_base(2,2) - ...
    d0(:,2)*face_base(1,1)*face_base(2,3) - d0(:,3)*face_base(1,2)*face_base(2,1);
t = (a - b) ./ c;

a = d0(:,1).*pts0(:,2)*face_base(2,3) + d0(:,2).*pts0(:,3)*face_base(2,1) + ...
    d0(:,3).*pts0(:,1)*face_base(2,2) - d0(:,1).*pts0(:,3)*face_base(2,2) - ...
    d0(:,2).*pts0(:,1)*face_base(2,3) - d0(:,3).*pts0(:,2)*face_base(2,1);
b = d0(:,1)*face_base(2,2)*face_point(3) + d0(:,2)*face_base(2,3)*face_point(1) + ...
    d0(:,3)*face_base(2,1)*face_point(2) - d0(:,1)*face_base(2,3)*face_point(2) - ...
    d0(:,2)*face_base(2,1)*face_point(3) - d0(:,3)*face_base(2,2)*face_point(1);
alpha = (a + b) ./ c;

a = d0(:,1).*pts0(:,2)*face_base(1,3) + d0(:,2).*pts0(:,3)*face_base(1,1) + ...
    d0(:,3).*pts0(:,1)*face_base(1,2) - d0(:,1).*pts0(:,3)*face_base(1,2) - ...
    d0(:,2).*pts0(:,1)*face_base(1,3) - d0(:,3).*pts0(:,2)*face_base(1,1);
b = d0(:,1)*face_base(1,2)*face_point(3) + d0(:,2)*face_base(1,3)*face_point(1) + ...
    d0(:,3)*face_base(1,1)*face_point(2) - d0(:,1)*face_base(1,3)*face_point(2) - ...
    d0(:,2)*face_base(1,1)*face_point(3) - d0(:,3)*face_base(1,2)*face_point(1);
beta = -(a + b) ./ c;

p = [pts0(:,1) + t.*d0(:,1), pts0(:,2) + t.*d0(:,2), pts0(:,3) + t.*d0(:,3)];
end