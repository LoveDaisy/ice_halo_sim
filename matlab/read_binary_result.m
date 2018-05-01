clear; clc; close all;

bin_file_path = '/Users/zhangjiajie/Documents/Ice Halo/codes/cpp/cmake-build-debug/bin/';
dir_fnames = dir([bin_file_path, 'directions_*.bin']);

wl_store = zeros(length(dir_fnames), 1);
for i = 1:length(dir_fnames)
    regexp_out = regexpi(dir_fnames(i).name, ...
        'directions_([\d.]+)_(\d).bin','tokens');
    wl_store(i) = str2double(regexp_out{1}{1});
end
wl_store = unique(wl_store);
spec_pts = length(wl_store);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Heatmap
heatmap_hw = 900;
heatmap_size = floor([1,1] * (2*heatmap_hw + 1));
heatmap_spec_raw = zeros(heatmap_size(1), heatmap_size(2), spec_pts);

cam_uv_offset = [0, 0];
% cam_proj = @(sph)camera_project(sph, [90, 89.9, 0], 120, ...
%     heatmap_size, 'Equiarea');
cam_proj = @(sph)camera_project(sph, [90, 43.5, 0], 40, ...
    heatmap_size, 'linear');

% wl_store = zeros(spec_pts, 1);
total_w = 0;
for i = 1:length(dir_fnames)
    dir_fname = dir_fnames(i).name;
    regexp_out = regexpi(dir_fname, ...
        'directions_([\d.]+)_(\d).bin','tokens');
    wl = str2double(regexp_out{1}{1});
    wl_idx = find(abs(wl - wl_store) < 0.01);
%     wl = str2double(dir_fname(end-8:end-4));
%     wl_store(i) = wl;

    fid = fopen([bin_file_path, dir_fname], 'rb');
    num = fread(fid, 1, 'int');
    read_num = 0;
    while read_num < num
        k = min(1000000, num - read_num);
        data = fread(fid, [4, k], 'float')';
        read_num = read_num + k;
        fprintf('Read %d/%d lines...\n', read_num, num);
        
        data_norm = sqrt(sum(data(:,1:3).^2, 2));
        data = data(abs(data_norm - 1) < 1e-6, :);
        total_w = total_w + sum(data(:,4));
        
        lon = atan2d(-data(:,2), -data(:,1));
        lat = asind(-data(:,3) ./ sqrt(sum(data(:,1:3).^2, 2)));
        xy = cam_proj([lon, lat]);
        xy = bsxfun(@plus, xy, cam_uv_offset);
        
        idx = 0 < xy(:,1) & xy(:,1) <= heatmap_size(2) & ...
            0 < xy(:,2) & xy(:,2) <= heatmap_size(1);
        if sum(idx) <= 0
            continue;
        end
        xy = xy(idx,:); tmp_w = data(idx, 4);
        
        tmp_heatmap = accumarray(sub2ind(heatmap_size, xy(:,2), xy(:,1)), tmp_w, ...
            [prod(heatmap_size), 1]);
        
        heatmap_spec_raw(:,:,wl_idx) = heatmap_spec_raw(:,:,wl_idx) + ...
            reshape(tmp_heatmap, heatmap_size);
    end
    fclose(fid);
end
total_w = total_w / spec_pts;

%%
heatmap_spec = heatmap_spec_raw;
% heatmap_spec = heatmap_spec + fliplr(heatmap_spec);
heatmap_spec = imfilter(heatmap_spec, fspecial('gaussian', 20, 1.2));
heatmap_spec = heatmap_spec / total_w * 4e3 * 2.0;
spec = [wl_store, reshape(heatmap_spec, [], spec_pts)'];

heatmap_rgb = spec_to_rgb(spec, 'Space', 'srgb', ...
    'Method', 'shrinktogray', 'Mix', true);
heatmap_rgb = reshape(heatmap_rgb, [heatmap_size, 3]);
% heatmap_rgb = heatmap_rgb.^.8;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize
figure(1); clf;
image(heatmap_rgb);
hold on;

xy = cam_proj([(0:360)', zeros(361,1)]);
xy = bsxfun(@plus, xy, cam_uv_offset);
idx = 0 < xy(:,1) & xy(:,1) <= heatmap_size(2) & ...
    0 < xy(:,2) & xy(:,2) <= heatmap_size(1);
xy = xy(idx,:);
plot(xy(:,1), xy(:,2), '.w');
axis ij;
axis equal; axis tight; axis off;