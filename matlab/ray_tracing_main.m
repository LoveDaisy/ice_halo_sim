clear; close all; clc;

% rng(2000);

heatmap_hw = 500;
heatmap = zeros(heatmap_hw*2+1, heatmap_hw*2+1);

num = 50;
rand_num = 100;
repeat_num = 1000;

ray_in = [-90, -27];

fprintf('Simulating for R...\n');
ray_out_store = cell(repeat_num, 1);
w_store = cell(repeat_num, 1);
parfor (i = 1:repeat_num, 3)
% for i = 1:repeat_num
%     axis_ori = [rand(rand_num,1)*360, zeros(rand_num,1)];
    axis_ori = [rand(rand_num,1)*360, acosd(1-2*rand(rand_num,1))-90];
%     axis_ori = [rand(rand_num,1)*360, randn(rand_num,1)*10+90];
%     idx = axis_ori(:,2) > 90;
%     axis_ori(idx,1) = axis_ori(idx,1) + 180;
%     axis_ori(idx,2) = 180 - axis_ori(idx,2);
    roll = rand(rand_num,1) * 360;
%     roll = zeros(rand_num,1);
    ratio = 5.0;
%     crst = generate_hex_pry_crystal(axis_ori, roll, ratio);
    crst = generate_hex_cyl_crystal(axis_ori, roll, ratio);
%     crst = generate_hexagonal_crystal(axis_ori, roll, ratio);

%     crst.n = 1.3166;    % g line, 435.8nm
%     crst.n = 1.3137;    % F line, 486.1nm
%     crst.n = 1.3112;    % e line, 546.1nm
%     crst.n = 1.3098;    % d line, 587.6nm
    crst.n = 1.3079;    % C line, 656.3nm
    [ray_out, w, lbl] = trace_ray(crst, repmat(ray_in, rand_num, 1), num);
    ray_out_store{i} = ray_out;
    w_store{i} = w;
end

str_prj_hov = 100;
sample_num = 0;
for i = 1:repeat_num
    ray_out = ray_out_store{i};
    w = w_store{i};
    xy = sph_to_xy_equiarea(-ray_out, str_prj_hov, heatmap_hw);
    idx = 0 < xy(:,1) & xy(:,1) <= heatmap_hw*2+1 & ...
        0 < xy(:,2) & xy(:,2) <= heatmap_hw*2+1 & ...
        sum(bsxfun(@minus, ray_out, ray_in).^2, 2) > 1e-6;
    xy = xy(idx,:); tmp_w = w(idx);
    for j = 1:length(tmp_w)
        heatmap(xy(j,2), xy(j,1)) = heatmap(xy(j,2), xy(j,1)) + tmp_w(j);
    end
    sample_num = sample_num + sum(idx);
end

%%
figure(1); clf;
tmp_img = (heatmap+fliplr(heatmap)).^.7;
tmp_img = tmp_img / prctile(tmp_img(:), 99.95) * .95;
% imshow(imfilter(tmp_img,fspecial('gaussian',40,1.5)));
imagesc(imfilter(tmp_img,fspecial('gaussian',40,1.5)));
hold on;
xy = sph_to_xy_equiarea(-ray_in, str_prj_hov, heatmap_hw);
% plot(xy(1), xy(2), 'yo', xy(1), xy(2), 'w+');
xy = sph_to_xy_equiarea([(0:360)', zeros(361,1)], ...
    str_prj_hov, heatmap_hw);
plot(xy(:,1), xy(:,2), 'w:');
axis equal; axis tight; axis off;
title(sprintf('samples: %d', sample_num));

