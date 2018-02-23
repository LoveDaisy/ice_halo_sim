function crst = generate_crystal(axis_ori, roll, ratio)
% This function generates a crystal given input parameters
% INPUT
%   axis_ori:   [lon,lat], in degree
%   roll:       roll angle, in degree
%   ratio:      height / radii
% OUTPUT
%   crst:

crst.axis_ori = axis_ori;
crst.roll = roll;
crst.ratio = ratio;

z = [cosd(axis_ori(2)) * cosd(axis_ori(1)), ...
    cosd(axis_ori(2)) * sind(axis_ori(1)), ...
    sind(axis_ori(2))];
zz = sqrt(1 - z(3)^2);
x = [-z(2), z(1), 0] / zz;
y = [-z(1) * z(3), -z(2) * z(3), 1 - z(3)^2] / zz;
local_axis = [x; y; z];

matR = axis_angle_to_matrix(z, roll);

crst.local_axis = local_axis * matR';
crst.normals = [sqrt(3)/2, 0.5, 0; 0, 1, 0; -sqrt(3)/2, 0.5, 0;
    -sqrt(3)/2, -0.5, 0; 0, -1, 0; sqrt(3)/2, -0.5, 0; 0, 0, 1; 0, 0, -1;] * matR';
crst.areas = [ratio*ones(6, 1); 3*sqrt(3); 3*sqrt(3)];

crst.init_pts = @(ray_vec, num)init_pts(crst, ray_vec, num);
crst.propagate = @(pts, face_id, ray_vec)propagate(crst, pts, face_id, ray_vec);
crst.reflect = @(pts, face_id, ray_vec)reflect(crst, pts, face_id, ray_vec);
crst.refract = @(pts, face_id, ray_vec)refract(crst, pts, face_id, ray_vec);
end


function [pts, face_id] = init_pts(crst, ray_vec, num)
% This function init first points
inc_ray_num = size(ray_vec, 1);
pts = nan(inc_ray_num*num, 3);
face_id = nan(inc_ray_num*num, 1);

theta = -ray_vec * crst.normals';
k = 0;
for i = 1:inc_ray_num
    areas = crst.areas .* theta(i, :);
    idx = find(areas > 0);
    areas = areas(idx);
    cum_areas = [0, cumsum(areas)];
    tmp_face_id = sum(bsxfun(@gt, rand(num, 1), cum_areas), 2);

    for j = 1:length(idx)
        current_face_id = idx(j);
        current_idx = find(tmp_face_id == current_face_id);
        current_ray_num = length(current_idx);

        if current_face_id == 1
            current_pts = rand(current_ray_num, 2) * [-0.5, sqrt(3)/2, 0; 0, 0, -crst.ratio];
            current_pts = bsxfun(@plus, current_pts, [1, 0, crst.ratio/2]);
        elseif current_face_id == 2
            current_pts = rand(current_ray_num, 2) * [-1, 0, 0; 0, 0, -crst.ratio];
            current_pts = bsxfun(@plus, current_pts, [0.5, sqrt(3)/2, crst.ratio/2]);
        elseif current_face_id == 3
            current_pts = rand(current_ray_num, 2) * [.5, sqrt(3)/2, 0; -crst.ratio];
            current_pts = bsxfun(@plus, current_pts, [-1, 0, crst.ratio/2]);
        elseif current_face_id == 4
            current_pts = rand(current_ray_num, 2) * [.5, -sqrt(3)/2, 0; -crst.ratio];
            current_pts = bsxfun(@plus, current_pts, [-1, 0, crst.ratio/2]);
        elseif current_face_id == 5
            current_pts = rand(current_ray_num, 2) * [1, 0, 0; 0, 0, -crst.ratio];
            current_pts = bsxfun(@plus, current_pts, [-.5, -sqrt(3), crst.ratio/2]);
        elseif current_face_id == 6
            current_pts = rand(current_ray_num, 2) * [-.5, -sqrt(3)/2, 0; 0, 0, -crst.ratio];
            current_pts = bsxfun(@plus, current_pts, [1, 0, crst.ratio/2]);
        else
            current_pts = rand(current_ray_num * 2, 2) * 2 - 1;
            idx = inpolygon(current_pts(:,1), current_pts(:,2), cosd(0:60:360)', sind(0:60:360)');
            current_pts = current_pts(idx, :);
            current_pts = current_pts(1:current_ray_num, :);
            if current_face_id == 6
                current_pts = [current_pts, crst.ratio/2 * ones(current_ray_num, 1)];
            else
                current_pts = [current_pts, -crst.ratio/2 * ones(current_ray_num, 1)];
            end
        end

        face_id(k+current_idx) = current_face_id;
        pts(k+current_idx, :) = current_pts;
        k = k + current_ray_num;
    end
end
end


function pts = propagate(crst, pts, face_id, ray_vec)
% This function propagate a ray to next face or exit this crystal.
end


function [ray_vec, w] = reflect(crst, pts, face_id, ray_vec)
% This function get the reflect ray vector and weight
end


function [ray_vec, w] = refract(crst, pts, face_id, ray_vec)
% This function get the refract ray vector and weight
end

