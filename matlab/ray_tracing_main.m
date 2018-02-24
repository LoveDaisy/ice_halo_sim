clear; close all; clc;

num = 100;
sim_n = 10000;
ray_out_store = nan(num*sim_n*5, 3);

heatmap_hw = 800;
str_prj_heatmap = zeros(heatmap_hw*2+1);
str_prj_hov = 100;

ray_in = [-90, -27];
k = 0; sample_num = 0;
for i = 1:sim_n
    axis_ori = [rand()*360, acosd(1-2*rand())-90];
%     axis_ori = [rand()*360, (rand()*2-1)*90];
    roll = (rand()*2-1)*5 + i*10;
    ratio = 5;
    crst = generate_crystal(axis_ori, roll, ratio);
    [ray_out, w] = trace_ray_parallel(crst, ray_in, num);
    ray_out_store(k+(1:length(w)), :) = [ray_out, w];
    k = k + length(w);

    if mod(i, 200) == 0
        xy = sph_to_xy_stereographic(-ray_out_store(:,1:2), str_prj_hov, heatmap_hw);
        idx = 0 < xy(:,1) & xy(:,1) <= heatmap_hw*2+1 & ...
            0 < xy(:,2) & xy(:,2) <= heatmap_hw*2+1 & ...
            sum(bsxfun(@minus, ray_out_store(:,1:2), ray_in).^2, 2) > 1e-6;
        xy = xy(idx,:); tmp_w = ray_out_store(idx, 3);
        for j = 1:length(tmp_w)
            str_prj_heatmap(xy(j,2), xy(j,1)) = str_prj_heatmap(xy(j,2), xy(j,1)) + tmp_w(j);
        end
        sample_num = sample_num + sum(idx);

        figure(1); clf;
        imagesc(imfilter((str_prj_heatmap+fliplr(str_prj_heatmap)).^.7,fspecial('gaussian',40,1.5)));
        hold on;
        xy = sph_to_xy_stereographic(-ray_in, str_prj_hov, heatmap_hw);
        plot(xy(1), xy(2), 'yo', xy(1), xy(2), 'w+');
        xy = sph_to_xy_stereographic([(0:360)', zeros(361,1)], ...
            str_prj_hov, heatmap_hw);
        plot(xy(:,1), xy(:,2), 'w:');
        axis equal; axis tight; axis off;
        title(sprintf('iter: %d. samples: %d', i, sample_num));
        drawnow;
    end
end
ray_out_store = ray_out_store(1:k, :);