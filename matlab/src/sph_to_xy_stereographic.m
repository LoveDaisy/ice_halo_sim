function xy = sph_to_xy_stereographic(sph, hov, max_r)
% This function convert spherical coordinates to Cartesian coordinates
% using stereographic projection.

str_prj_r = max_r / 2 / tand(hov / 2);
r = 2 * str_prj_r .* tand((90 - sph(:, 2))/2);
x = floor(r .* cosd(sph(:, 1)) + max_r);
y = floor(r .* sind(sph(:, 1)) + max_r);
xy = [x, y];
end