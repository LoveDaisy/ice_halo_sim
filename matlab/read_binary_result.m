clear; clc; close all;

bin_file_path = '/Users/zhangjiajie/Documents/Ice Halo/codes/cpp/cmake-build-debug/bin/';
dir_fnames = dir([bin_file_path, 'directions_*.bin']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Heatmap
str_prj_hov = 100;
heatmap_hw = 500;
heatmap_size = [1,1] * (2*heatmap_hw + 1);
heatmap_spec_raw = zeros(heatmap_size(1), heatmap_size(2), length(dir_fnames));

spec_pts = length(dir_fnames);

wl_store = zeros(spec_pts, 1);
for i = 1:spec_pts
    dir_fname = dir_fnames(i).name;
    wl = str2double(dir_fname(end-8:end-4));
    wl_store(i) = wl;

    fid = fopen([bin_file_path, dir_fname], 'rb');
    num = fread(fid, 1, 'int');
    read_num = 0;
    total_w = 0;
    while read_num < num
        k = min(1000000, num - read_num);
        data = fread(fid, [4, k], 'float')';
        read_num = read_num + k;
        total_w = total_w + sum(data(:,4));
        fprintf('Read %d/%d lines...\n', read_num, num);
        
        lon = atan2d(-data(:,2), -data(:,1));
        lat = asind(-data(:,3) ./ sqrt(sum(data(:,1:3).^2, 2)));
        xy = sph_to_xy_equiarea([lon, lat], str_prj_hov, heatmap_hw);
        
        idx = 0 < xy(:,1) & xy(:,1) <= heatmap_hw*2+1 & ...
            0 < xy(:,2) & xy(:,2) <= heatmap_hw*2+1;
        xy = xy(idx,:); tmp_w = data(idx, 4);
        
        tmp_heatmap = accumarray(sub2ind(heatmap_size, xy(:,2), xy(:,1)), tmp_w, ...
            [prod(heatmap_size), 1]);
        
        heatmap_spec_raw(:,:,i) = heatmap_spec_raw(:,:,i) + reshape(tmp_heatmap, heatmap_size);
    end
    fclose(fid);
end

%%
heatmap_spec = heatmap_spec_raw + fliplr(heatmap_spec_raw);
heatmap_spec = imfilter(heatmap_spec, fspecial('gaussian', 20, 1.2));
heatmap_spec = heatmap_spec / max(heatmap_spec(:)) * .6;
spec = [wl_store, reshape(heatmap_spec, [], spec_pts)'];

heatmap_rgb = spec_to_rgb(spec, 'Space', 'srgb', ...
    'Method', 'shrinktogray', 'MaxY', .1, 'Mix', true);
heatmap_rgb = reshape(heatmap_rgb, [heatmap_size, 3]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize
figure(1); clf;
hold on;
tmp_img = heatmap_rgb;
image(tmp_img);
xy = sph_to_xy_equiarea([(0:360)', zeros(361,1)], ...
    str_prj_hov, heatmap_hw);
plot(xy(:,1), xy(:,2), 'w:');
axis equal; axis tight; axis off;