function uv = camera_project(sph, cam_rot, hov, img_size, method)
% This function convert spherical coordinates to image plane Cartesian coordinates
% INPUT
%   sph:        [lon, lat], spherical coordinates
%   cam_rot:    [lon, lat, roll], the direction to which lens point
%   hov:        Half field of view. Measured horizontally
%   img_size:   [hei, wid]
%   method:     {'Linear', 'Equiarea', 'Stereo'}
% OUTPUT
%   uv:         [u, v], image plane Cartesian coordinates
%
% The image coordinate frame
% z-axis is the lens optical axis pointing outward of camera.
% x-axis is the same as image x-axis.
% y-axis is the same as image y-axis, pointing downside.

lon = cam_rot(1); lat = cam_rot(2); roll = cam_rot(3);
ax_z = [cosd(lon) * cosd(lat), sind(lon) * cosd(lat), sind(lat)];
ax_x = [sind(lon), -cosd(lon), 0];
ax_y = cross(ax_z, ax_x);
matR = [ax_x; ax_y; ax_z];
matR = matR * axis_angle_to_matrix(ax_z, roll)';

x = cosd(sph(:,1)) .* cosd(sph(:,2));
y = sind(sph(:,1)) .* cosd(sph(:,2));
z = sind(sph(:,2));
xyz = [x, y, z] * matR';

if strcmpi(method, 'linear')
    uv = bsxfun(@times, xyz(:,1:2), 1./xyz(:,3));
    uv = uv * img_size(2) / 2 / tand(hov);
    uv = bsxfun(@plus, uv, wrev(img_size / 2));
    uv = floor(uv);
    uv(xyz(:,3) < 0, :) = nan;
else
    lon = atan2d(xyz(:,2), xyz(:,1));
    lat = asind(xyz(:,3) ./ sqrt(sum(xyz.^2, 2)));

    if strcmpi(method, 'equiarea')
        str_prj_r = img_size(2) / 4 / sind(hov / 2);
        r = 2 * str_prj_r .* sind((90 - lat)/2);
    elseif strcmpi(method, 'stereo')
        str_prj_r = img_size(2) / 4 / tand(hov / 2);
        r = 2 * str_prj_r .* tand((90 - lat)/2);
    else
        error('Method not recognized!');
    end
    u = r .* cosd(lon) + img_size(2)/2;
    v = r .* sind(lon) + img_size(1)/2;
    uv = floor([u, v]);
end
end