function xy = sph_to_xy_equiarea(sph, hov, max_r)
str_prj_r = max_r / 2 / sind(hov / 2);
r = 2 * str_prj_r .* sind((90 - sph(:, 2))/2);
x = floor(r .* cosd(sph(:, 1)) + max_r);
y = floor(r .* sind(sph(:, 1)) + max_r);
xy = [x, y];
end