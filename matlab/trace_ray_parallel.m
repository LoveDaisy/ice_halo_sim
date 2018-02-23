function [ray_out, w] = trace_ray_parallel(geometry, ray_in, num)
% This function traces parallel incident rays, and return exit rays
% INPUT
%   geometry:
%       .local_axis:    each row is a local axes
%       .propagate:     a function handle. propagate(p, face_id, d) returns
%                       the next intersection point starts from p
%                       with direction d.
%       .refract:       a function handle. refract(p, face_id, d) returns
%                       new ray directions.
%       .reflect:       a function handle. reflect(p, face_id, d) returns
%                       new ray directions.
%       .init_pts:      a function handle. init_pts(num) returns
%                       num points on geometry surfaces, distributed
%                       evenly seeing along the incident ray.
%   ray_in:             [lon, lat]
%   num:                Monte-Carlo simulation number.

matR = geometry.local_axis;

ray_vec = [cosd(ray_in(:,2)).*cosd(ray_in(:,1)), ...
    cosd(ray_in(:,2)).*sind(ray_in(:,1)), sind(ray_in(:,2))];
ray_vec = ray_vec * matR';

[pts, face_id] = geometry.init_pts(ray_vec, num);

ray_out_vec = zeros(num*10, 3);
w = zeros(num*10, 3);
ray_w = 1;
k = 0; ref_num = 0;
while true
    [ray_reflect_vec, reflect_w] = geometry.reflect(pts, face_id, ray_vec);
    [ray_refract_vec, refract_w] = geometry.refract(pts, face_id, ray_vec);
    reflect_pts = geometry.propagate(pts, face_id, ray_reflect_vec);
    refract_pts = geometry.propagate(pts, face_id, ray_refract_vec);
    reflect_w = reflect_w .* ray_w;
    refract_w = refract_w .* ray_w;

    idx = isnan(reflect_pts(:,1));
    nn = sum(idx);
    ray_out_vec(k+1:k+nn, :) = ray_reflect_vec(idx, :);
    w(k+1:k+nn) = reflect_w(idx);
    k = k + nn;
    reflect_pts = reflect_pts(~idx & reflect_w > w_th, :);
    reflect_w = reflect_w(~idx & reflect_w > w_th);

    idx = isnan(refract_pts(:,1));
    nn = sum(idx);
    ray_out_vec(k+1:k+nn, :) = ray_refract_vec(idx, :);
    w(k+1:k+nn) = refract_w(idx);
    k = k + nn;
    refract_pts = refract_pts(~idx & refract_w > w_th, :);
    refract_w = refract_w(~idx & refract_w > w_th);

    pts = [reflect_pts; refract_pts];
    ray_w = [reflect_w; refract_w];
    ray_vec = [ray_reflect_vec; ray_refract_vec];
    ref_num = ref_num + 1;

    if isempty(ray_w) || ref_num > 6
        break;
    end
end

ray_out_vec = ray_out_vec(1:k, :);
ray_out_vec = ray_out_vec * matR;
w = w(1:k);
ray_out = [atan2d(ray_out_vec(:,2), ray_out_vec(:,1)), asind(ray_out_vec(:,3))];
end