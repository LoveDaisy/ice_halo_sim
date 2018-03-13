clear; clc; close all;

bin_file_path = '/Users/zhangjiajie/Documents/Ice Halo/codes/cpp/cmake-build-debug/';
dir_fname = 'directions.bin';
geo_fname = 'geometry.bin';
ray_fname = 'rays.bin';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Heatmap
str_prj_hov = 100;
heatmap_hw = 500;
heatmap_size = [1,1] * (2*heatmap_hw + 1);
heatmap = zeros(2*heatmap_hw+1);

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
    heatmap = heatmap + reshape(tmp_heatmap, heatmap_size);
end
fclose(fid);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Geometry
% fid = fopen([bin_file_path, geo_fname], 'rb');
% num = fread(fid, 1, 'int');
% vtx = fread(fid, [3, num], 'float')';
% num = fread(fid, 1, 'int');
% vtx_idx = fread(fid, [3, num], 'int')';
% fclose(fid);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Rays
% fid = fopen([bin_file_path, ray_fname], 'rb');
% ray_num = fread(fid, 1, 'int');
% rays = cell(num, 1);
% for i = 1:ray_num
%     num = fread(fid, 1, 'int');
%     ray = fread(fid, [8, num], 'float')';
%     rays{i} = ray;
% end
% fclose(fid);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize
figure(1); clf;
hold on;
imagesc((imfilter(heatmap, fspecial('gaussian', 20, 1.5))));
xy = sph_to_xy_equiarea([(0:360)', zeros(361,1)], ...
    str_prj_hov, heatmap_hw);
plot(xy(:,1), xy(:,2), 'w:');
axis equal; axis tight; axis off;