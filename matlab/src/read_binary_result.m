clear; clc; close all;

% bin_file_path = '/Volumes/ZJJ-4TB/Ice Halo Data/Hailar/';
bin_file_path = '/Users/zhangjiajie/Codes/Ice Halo/cpp/cmake-build-debug/';
dir_fnames = dir([bin_file_path, 'directions_*.bin']);

wl_store = zeros(length(dir_fnames), 1);
for i = 1:length(dir_fnames)
    regexp_out = regexpi(dir_fnames(i).name, ...
        'directions_([\d.]+)_(\d+).bin','tokens');
    wl_store(i) = str2double(regexp_out{1}{1});
end
wl_store = unique(wl_store);
spec_pts = length(wl_store);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Heatmap
heatmap_hw = 900;
heatmap_size = floor([1,1] * (2*heatmap_hw + 1));
heatmap_spec_raw = zeros(heatmap_size(1), heatmap_size(2), spec_pts);
heatmap_spec_cnt = zeros(size(heatmap_spec_raw));

cam_uv_offset = [0, 0];
% cam_proj = @(sph)camera_project(sph, [90, 89.99, 0], 120, ...
%     heatmap_size, 'Equiarea');
cam_proj = @(sph)camera_project(sph, [90, 10, 0], 40, ...
    heatmap_size, 'linear');

w11 = 0; w21 = 0;
block_read_lines = 1000000;
total_w = 0;
for i = 1:length(dir_fnames)
    fprintf('Reading #%d/%d...\n', i, length(dir_fnames));
    dir_fname = dir_fnames(i).name;
    regexp_out = regexpi(dir_fname, ...
        'directions_([\d.]+)_(\d+).bin','tokens');
    wl = str2double(regexp_out{1}{1});
    wl_idx = find(abs(wl - wl_store) < 0.01);

    fid = fopen([bin_file_path, dir_fname], 'rb');
    read_num = block_read_lines;
    while read_num >= block_read_lines
%         k = min(block_read_lines, num - read_num);
        data = fread(fid, [4, block_read_lines], 'float')';
        read_num = size(data, 1);
%         fprintf('Read %d lines...\n', read_num);
        if isempty(data)
            break;
        end
        
        data_norm = sqrt(sum(data(:,1:3).^2, 2));
        data = data(abs(data_norm - 1) < 1e-6, :);
        
        % Add atmosphere scattering
        data(:,1:3) = data(:,1:3) + randn(size(data(:,1:3))) * 0.015;
        data_norm = sqrt(sum(data(:,1:3).^2, 2));
        data(:,1:3) = bsxfun(@times, data(:,1:3), 1./data_norm);
        
        total_w = total_w + sum(data(:,4));
        
        lon = atan2d(-data(:,2), -data(:,1));
        lat = asind(-data(:,3) ./ sqrt(sum(data(:,1:3).^2, 2)));
        xy1 = cam_proj([lon, lat]);
        xy1 = bsxfun(@plus, xy1, cam_uv_offset);
        
%         % For debug
%         xy0 = cam_proj([lon, lat]);
%         xy0 = bsxfun(@plus, xy0, cam_uv_offset);
%         tmp_idx = abs(xy1(:,1) - 898) + abs(xy1(:,2) - 1127) < 4;
%         w11 = w11 + sum(data(tmp_idx, 4));
%         w11_cnt = sum(tmp_idx);
%         tmp_idx = abs(xy1(:,1) - 898) + abs(xy1(:,2) - 1308) < 4;
%         w21_cnt = sum(tmp_idx);
%         w21 = w21 + sum(data(tmp_idx, 4));
%         fprintf('w1(%d):%f, w2(%d):%f\n', w11_cnt, w11, w21_cnt, w21);
        
        idx = 0 < xy1(:,1) & xy1(:,1) <= heatmap_size(2) & ...
            0 < xy1(:,2) & xy1(:,2) <= heatmap_size(1) & lat > 0 & ...
            ~any(isnan(xy1), 2);
%         idx = 0 < xy1(:,1) & xy1(:,1) <= heatmap_size(2) & ...
%             0 < xy1(:,2) & xy1(:,2) <= heatmap_size(1) & ...
%             ~any(isnan(xy1), 2);
        if sum(idx) <= 0
            continue;
        end
        xy1 = xy1(idx,:); tmp_w = data(idx, 4);
        
        tmp_heatmap = accumarray(sub2ind(heatmap_size, xy1(:,2), xy1(:,1)), tmp_w, ...
            [prod(heatmap_size), 1]);
        tmp_cnt = accumarray(sub2ind(heatmap_size, xy1(:,2), xy1(:,1)), ones(size(tmp_w)), ...
            [prod(heatmap_size), 1]);
        
        heatmap_spec_raw(:,:,wl_idx) = heatmap_spec_raw(:,:,wl_idx) + ...
            reshape(tmp_heatmap, heatmap_size);
        heatmap_spec_cnt(:,:,wl_idx) = heatmap_spec_cnt(:,:,wl_idx) + ...
            reshape(tmp_cnt, heatmap_size);
    end
    fclose(fid);
end
total_w = total_w / spec_pts;

%%
heatmap_spec = heatmap_spec_raw;
% heatmap_spec = imfilter(heatmap_spec, fspecial('gaussian', 20, 1.1));
heatmap_spec = heatmap_spec / total_w * 4e3 * 3;
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

%%
bar_theta = (0:40)';
bar_center = [90, 1];
xyz = [-sind(bar_theta) * sind(bar_center(1)), ...
    cosd(bar_theta) * cosd(bar_center(2)) * sind(bar_center(1)), ...
    cosd(bar_theta) * sind(bar_center(2))];
xy1 = cam_proj([atan2d(xyz(:,2), xyz(:,1)), asind(xyz(:,3) ./ sqrt(sum(xyz.^2, 2)))]);
idx = 0 < xy1(:,1) & xy1(:,1) <= heatmap_size(2) & ...
    0 < xy1(:,2) & xy1(:,2) <= heatmap_size(1);
xy1 = xy1(idx,:);

bar_theta = -(0:40)';
xyz = [-sind(bar_theta) * sind(bar_center(1)), ...
    cosd(bar_theta) * cosd(bar_center(2)) * sind(bar_center(1)), ...
    cosd(bar_theta) * sind(bar_center(2))];
xy2 = cam_proj([atan2d(xyz(:,2), xyz(:,1)), asind(xyz(:,3) ./ sqrt(sum(xyz.^2, 2)))]);
idx = 0 < xy2(:,1) & xy2(:,1) <= heatmap_size(2) & ...
    0 < xy2(:,2) & xy2(:,2) <= heatmap_size(1);
xy2 = xy2(idx,:);

bar_theta = (0:40)';
xyz = [zeros(size(bar_theta)), ...
    cosd(bar_theta + bar_center(2)) * sind(bar_center(1)), ...
    cosd(bar_theta) * sind(bar_center(2)) + sind(bar_theta) * sind(bar_center(2))];
xy3 = cam_proj([atan2d(xyz(:,2), xyz(:,1)), asind(xyz(:,3) ./ sqrt(sum(xyz.^2, 2)))]);
idx = 0 < xy3(:,1) & xy3(:,1) <= heatmap_size(2) & ...
    0 < xy3(:,2) & xy3(:,2) <= heatmap_size(1);
xy3 = xy3(idx,:);

bar_theta = -(0:40)';
xyz = [zeros(size(bar_theta)), ...
    cosd(bar_theta + bar_center(2)) * sind(bar_center(1)), ...
    cosd(bar_theta) * sind(bar_center(2)) + sind(bar_theta) * sind(bar_center(2))];
xy4 = cam_proj([atan2d(xyz(:,2), xyz(:,1)), asind(xyz(:,3) ./ sqrt(sum(xyz.^2, 2)))]);
idx = 0 < xy4(:,1) & xy4(:,1) <= heatmap_size(2) & ...
    0 < xy4(:,2) & xy4(:,2) <= heatmap_size(1);
xy4 = xy4(idx,:);

heatmap_rgb_bar = heatmap_rgb;
heatmap_rgb_bar(heatmap_hw, :, :) = 0.4;
heatmap_rgb_bar(:, heatmap_hw, :) = 0.4;
for i = 1:size(xy1,1)
    heatmap_rgb_bar(xy1(i,2)+(-5:5), xy1(i,1), :) = 0.4;
end
for i = 1:size(xy2,1)
    heatmap_rgb_bar(xy2(i,2)+(-5:5), xy2(i,1), :) = 0.4;
end
for i = 1:size(xy3,1)
    heatmap_rgb_bar(xy3(i,2), xy3(i,1)+(-5:5), :) = 0.4;
end
for i = 1:size(xy4,1)
    heatmap_rgb_bar(xy4(i,2), xy4(i,1)+(-5:5), :) = 0.4;
end

for i = 1:5:size(xy1,1)
    heatmap_rgb_bar(xy1(i,2)+(-10:10), xy1(i,1), :) = 0.4;
end
for i = 1:5:size(xy2,1)
    heatmap_rgb_bar(xy2(i,2)+(-10:10), xy2(i,1), :) = 0.4;
end
for i = 1:5:size(xy3,1)
    heatmap_rgb_bar(xy3(i,2), xy3(i,1)+(-10:10), :) = 0.4;
end
for i = 1:5:size(xy4,1)
    heatmap_rgb_bar(xy4(i,2), xy4(i,1)+(-10:10), :) = 0.4;
end

% figure(2); clf;
% image(heatmap_rgb_bar);
% 
% axis ij;
% axis equal; axis tight; axis off;


%%
% Write
img_file_path = '/Users/zhangjiajie/Documents/Ice Halo/';
% imwrite(uint16(heatmap_rgb_bar*65535), [img_file_path, 'test_bar.tiff']);
imwrite(uint16(heatmap_rgb*65535), [img_file_path, 'test.tiff']);
% imwrite(uint8(heatmap_rgb_bar*255), [img_file_path, 'test_bar.jpg']);
imwrite(uint8(heatmap_rgb*255), [img_file_path, 'test.jpg']);
