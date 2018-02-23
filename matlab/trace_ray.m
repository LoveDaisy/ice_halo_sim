function [ray_out, w] = trace_ray(crst, ray_in, mc_num)
% This function trace income rays, both refractively and reflectively.
% INPUT
%   crst:
%   ray_in:     incoming ray direction. [lon, lat]
%   mc_num:
% OUTPUT
%   ray_out:    outcoming ray direction. [lon, lat]
%   w:          weight for each ray direction.

n = 1.33;

ray_in_n = [cosd(ray_in(:,2)).*cosd(ray_in(:,1)), cosd(ray_in(:,2)).*sind(ray_in(:,1)), sind(ray_in(:,2))];
cos_theta = (-ray_in_n) * crst.normals';

faces_proj_area = bsxfun(@times, crst.areas, cos_theta);
faces_vis = cos_theta > 0;

ray_out = nan(size(ray_in, 1) * mc_num, 2);
w = nan(size(ray_in, 1) * mc_num, 1);

k = 0;
for i = 1:size(ray_in, 1)
    face_id = find(faces_vis(i, :));
    face_w = faces_proj_area(i, faces_vis(i, :));
    face_w = face_w / sum(face_w);

    for j = 1:length(face_w)
        [tmp_ray_out, tmp_w] = crst_ray(crst, ray_in_n(i, :), face_id(j), n, floor(mc_num * face_w(j)));
        ray_out(k+1:k+length(tmp_w), :) = tmp_ray_out;
        w(k+1:k+length(tmp_w)) = tmp_w;
        k = k + length(tmp_w);
    end
end

idx = ~isnan(w);
ray_out = ray_out(idx, :);
w = w(idx);
end


function [ray_out, w] = crst_ray(crst, ray_in_n, n, face_id, num)
matR = inv(crst.local_axis);
ray_in_n = ray_in_n * matR;

if face_id == 1
    pts = bsxfun(@times, rand(num, 1), [-.5, sqrt(3)/2, 0]) + ...
        bsxfun(@times, rand(num, 1), [0, 0, -crst.ratio/2]);
    pts = bsxfun(@plus, pts, [1, 0, crst.ratio/2]);
elseif face_id == 2
    pts = bsxfun(@times, rand(num, 1), [-1, 0, 0]) + ...
        bsxfun(@times, rand(num, 1), [0, 0, -crst.ratio/2]);
    pts = bsxfun(@plus, pts, [.5, sqrt(3)/2, crst.ratio/2]);
elseif face_id == 3
    pts = bsxfun(@times, rand(num, 1), [.5, sqrt(3)/2, 0]) + ...
        bsxfun(@times, rand(num, 1), [0, 0, -crst.ratio/2]);
    pts = bsxfun(@plus, pts, [-1, 0, crst.ratio/2]);
elseif face_id == 4
    pts = bsxfun(@times, rand(num, 1), [.5, -sqrt(3)/2, 0]) + ...
        bsxfun(@times, rand(num, 1), [0, 0, -crst.ratio/2]);
    pts = bsxfun(@plus, pts, [-1, 0, crst.ratio/2]);
elseif face_id == 5
    pts = bsxfun(@times, rand(num, 1), [1, 0, 0]) + ...
        bsxfun(@times, rand(num, 1), [0, 0, -crst.ratio/2]);
    pts = bsxfun(@plus, pts, [-.5, -sqrt(3)/2, crst.ratio/2]);
elseif face_id == 6
    pts = bsxfun(@times, rand(num, 1), [-.5, -sqrt(3)/2, 0]) + ...
        bsxfun(@times, rand(num, 1), [0, 0, -crst.ratio/2]);
    pts = bsxfun(@plus, pts, [1, 0, crst.ratio/2]);
elseif face_id >= 7
    pts = rand(2*num, 2) * 2 - 1;
    pts = pts(inpolygon(cosd(0:60:360), sind(0:60:360), pts(:,1), pts(:,2)), :);
    pts = pts(1:num, :);
end

cc = -ray_in_n * crst.normals(face_id, :)';
rr = 1 / n;
ray_refract_n = ray_in_n * rr + (cc * rr - sqrt(1 - (1 - cc.^2) * rr^2)) * crst.normals(face_id, :);
ray_reflect_n = ray_in_n + 2 * cc .* crst.normals(face_id, :);

reflect_ratio = 0;
end


function pts = ray_trans(crst, ray_n, pts)
end


function ray_out_n = ray_ref(crst, ray_n, pts)
end