function crst = generate_hexagonal_crystal(axis_ori, roll, ratio)
% This function generates a crystal given input parameters
% INPUT
%   axis_ori:   [lon,lat], in degree
%   roll:       roll angle, in degree
%   ratio:      height / radii
% OUTPUT
%   crst:

assert(size(axis_ori,1) == size(roll,1));
assert(length(ratio) == 1);

ori_num = size(axis_ori, 1);

crst.axis_ori = axis_ori;
crst.roll = roll;
crst.ratio = ratio;

crst.local_axis = zeros(3, 3, ori_num);

for i = 1:ori_num
    z = [cosd(axis_ori(i,2)) * cosd(axis_ori(i,1)), ...
        cosd(axis_ori(i,2)) * sind(axis_ori(i,1)), ...
        sind(axis_ori(i,2))];
    zz = sqrt(1 - z(3)^2);
    x = [-z(2), z(1), 0] / zz;
    y = [-z(1) * z(3), -z(2) * z(3), 1 - z(3)^2] / zz;
    tmp_axis = [x; y; z];

    matR = axis_angle_to_matrix(z, roll(i));

    crst.local_axis(:,:,i) = tmp_axis * matR';
end

crst.normals = [sqrt(3)/2, 0.5, 0; 0, 1, 0; -sqrt(3)/2, 0.5, 0;
    -sqrt(3)/2, -0.5, 0; 0, -1, 0; sqrt(3)/2, -0.5, 0; 0, 0, 1; 0, 0, -1;];
crst.face_base_vec = [-.5, sqrt(3)/2, 0, 0, 0, -ratio;
                      -1, 0, 0, 0, 0, -ratio;
                      .5, sqrt(3)/2, 0, 0, 0, -ratio;
                      .5, -sqrt(3)/2, 0, 0, 0, -ratio;
                      1, 0, 0, 0, 0, -ratio;
                      -.5, -sqrt(3)/2, 0, 0, 0, -ratio;
                      1, 0, 0, 0, 1, 0;
                      1, 0, 0, 0, 1, 0];
crst.face_base_point = [1, 0, ratio/2;
                        .5, sqrt(3)/2, ratio/2;
                        -1, 0, ratio/2;
                        -1, 0, ratio/2;
                        -.5, -sqrt(3)/2, ratio/2;
                        1, 0, ratio/2;
                        0, 0, ratio/2;
                        0, 0, -ratio/2];
crst.areas = [ratio*ones(6, 1); 3*sqrt(3); 3*sqrt(3)];

crst.init_pts = @(ray_vec, num)init_pts(crst, ray_vec, num);
crst.propagate = @(pts, face_id, ray_vec)propagate(crst, pts, face_id, ray_vec);
crst.reflect = @(face_id, ray_vec)reflect(crst, face_id, ray_vec);
crst.refract = @(face_id, ray_vec)refract(crst, face_id, ray_vec);
end


function [pts, face_id, lbl] = init_pts(crst, ray_vec0, num)
% This function init first points
inc_ray_num = size(ray_vec0, 1);
pts = nan(inc_ray_num*num, 3);
face_id = nan(inc_ray_num*num, 1);
lbl = kron((1:length(ray_vec0))', ones(num, 1));

theta = -ray_vec0 * crst.normals';
for i = 1:inc_ray_num
    areas = crst.areas' .* theta(i, :);
    face_ids = find(areas > 0);
    areas = areas(face_ids);
    cum_areas = [0, cumsum(areas)] / sum(areas);
    tmp_face_id = sum(bsxfun(@gt, rand(num, 1), cum_areas), 2);

    for j = 1:length(face_ids)
        current_face_id = face_ids(j);
        current_idx = find(face_ids(tmp_face_id) == current_face_id);
        current_ray_num = length(current_idx);

        if current_face_id <= 6
            current_pts = rand(current_ray_num, 2) * ...
                reshape(crst.face_base_vec(current_face_id, :), [], 2)';
            current_pts = bsxfun(@plus, current_pts, crst.face_base_point(current_face_id, :));
        else
            while true
                tmp_pts = rand(current_ray_num * 2, 2) * 2 - 1;
                idx = inpolygon(tmp_pts(:,1), tmp_pts(:,2), cosd(0:60:360)', sind(0:60:360)');
                if sum(idx) >= current_ray_num
                    break;
                end
            end
            tmp_pts = tmp_pts(idx, :);
            tmp_pts = tmp_pts(1:current_ray_num, :);
            if current_face_id == 7
                current_pts = [tmp_pts, crst.ratio/2 * ones(current_ray_num, 1)];
            else
                current_pts = [tmp_pts, -crst.ratio/2 * ones(current_ray_num, 1)];
            end
        end

        face_id((i-1)*num + current_idx) = current_face_id;
        pts((i-1)*num + current_idx, :) = current_pts;
    end
end
idx = ~isnan(face_id);
pts = pts(idx, :);
face_id = face_id(idx);
lbl = lbl(idx, :);
end


function [pts, face_id] = propagate(crst, pts0, face_id0, ray_vec)
% This function propagate a ray to next face or exit this crystal.
pts_num = size(pts0, 1);

pts = nan(pts_num, 3);
t = inf(pts_num, 1);
face_id = nan(pts_num, 1);

normals = crst.normals(face_id0, :);
cos_theta = sum(ray_vec .* normals, 2);
valid_idx = find(cos_theta < 0);

if isempty(valid_idx)
    return
end

for i = 1:8
    face_base = reshape(crst.face_base_vec(i, :), [], 2)';
    face_point = crst.face_base_point(i, :);
    [tmp_p, tmp_t, tmp_alpha, tmp_beta] = intersect_line_face(pts0(valid_idx, :), ...
        ray_vec(valid_idx, :), face_base, face_point);
    idx = tmp_t < t(valid_idx) & tmp_t > 1e-6;
    if sum(idx) < 1
        continue;
    end
    if i > 6
        idx = idx & inpolygon(tmp_p(:,1), tmp_p(:,2), cosd(0:60:360)', sind(0:60:360)');
    else
        idx = idx & (0 <= tmp_alpha & tmp_alpha <= 1) & (0 <= tmp_beta & tmp_beta <= 1);
    end
    if sum(idx) < 1 || isempty(idx)
        continue;
    end
    face_id(valid_idx(idx)) = i;
    pts(valid_idx(idx), :) = tmp_p(idx, :);
    t(valid_idx(idx)) = tmp_t(idx);
end
end


function [ray_vec, w] = reflect(crst, face_id, ray_vec)
% This function get the reflect ray vector and weight

assert(size(face_id, 1) == size(ray_vec, 1) || size(ray_vec, 1) == 1);

normals = crst.normals(face_id, :);
cos_theta = sum(bsxfun(@times, normals, ray_vec), 2);
inc_angle = acosd(abs(cos_theta));

in_crst = cos_theta > 0;
n1 = in_crst * 1.33 + (~in_crst) * 1;
n2 = in_crst * 1 + (~in_crst) * 1.33;
normals(in_crst, :) = -normals(in_crst, :);

[Rs, Rp] = calculate_reflect_ratio(inc_angle, n1, n2);
w = (Rs + Rp) / 2;
ray_vec = bsxfun(@plus, ray_vec, bsxfun(@times, 2 * abs(cos_theta), normals));
end


function [ray_vec, w] = refract(crst, face_id, ray_vec)
% This function get the refract ray vector and weight

assert(size(face_id, 1) == size(ray_vec, 1) || size(ray_vec, 1) == 1);

normals = crst.normals(face_id, :);
cos_theta = sum(bsxfun(@times, normals, ray_vec), 2);
inc_angle = acosd(abs(cos_theta));

in_crst = cos_theta > 0;
n1 = in_crst * 1.33 + (~in_crst) * 1;
n2 = in_crst * 1 + (~in_crst) * 1.33;
rr = n1 ./ n2;
normals(in_crst, :) = -normals(in_crst, :);

[Rs, Rp] = calculate_reflect_ratio(inc_angle, n1, n2);
w = 1 - (Rs + Rp) / 2;
d = max(1 - rr.^2.*(1 - cos_theta.^2), 0);
ray_vec = bsxfun(@times, rr, ray_vec) + ...
    bsxfun(@times, (rr .* abs(cos_theta) - sqrt(d)), normals);
% ray_vec = ray_vec(w > 0, :);
% w = w(w > 0);
end

