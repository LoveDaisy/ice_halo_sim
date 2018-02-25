clear; close all; clc;

num = 50;
ray_in = [90, -27];

axis_ori = [0, 0];
roll = 20;
ratio = 5;

rng(5000);

% crst = generate_hexagonal_crystal(axis_ori, roll, ratio);
crst = generate_hex_cyl_crystal(axis_ori, roll, ratio);
[ray_out, w] = trace_ray(crst, ray_in, num);

figure(1); clf;
scatter(ray_out(:,1), ray_out(:,2), w*50);