clear; close all; clc;

num = 100;
yr = 50;
xr = 0.2;
alpha = (1 - xr + xr*yr) / yr;
beta = 1 - xr + xr*yr;
th = (xr * yr) / (2 + 2 * xr * (-1 + yr));

bin_num = 1000;
edges = linspace(0, 1, bin_num+1);
bin_centers = (edges(1:end-1) + edges(2:end)) / 2;
bin_val = zeros(size(bin_centers));
bin_counts = zeros(size(bin_centers));
x_counts = zeros(size(bin_counts));

prior_num = 103;
prior_p = ones(prior_num, 1);
prior_cnt = zeros(prior_num, 1);
prior_w = zeros(size(prior_cnt));

ref = 1/beta * ones(size(bin_centers)) + (yr-1)/beta * (abs(bin_centers - 0.5) < xr/2);

total_num = 100e4;
iter_num = 0;
iter_disp = 0;
disp_gap = 1000;
while sum(bin_counts) < total_num
    iter_num = iter_num + 1;
    iter_disp = iter_disp + 1;
    x_store = [];
    while length(x_store) < num
        tmp_x = rand(5*num, 1);
        select = rand(size(tmp_x));
        tmp_x = tmp_x(select < prior_p(floor(tmp_x * prior_num) + 1));
        x_store = [x_store; tmp_x];
    end
    [tmp_cnts, ~, ~] = histcounts(x_store, edges);
    x_counts = x_counts + tmp_cnts;
    
    w_store = max(abs(x_store - 0.5)*alpha, abs(x_store - 0.5)*beta+(1-beta)*0.5) ...
        .* sign(x_store - 0.5) + 0.5;
    y_store = randn(size(x_store))*.2 + ...
        (1/((yr-1)*xr+1) + (yr-1)/((yr-1)*xr+1)*(abs(x_store - 0.5)<th));
    
    [tmp_cnts, ~, idx] = histcounts(w_store, edges);
    bin_counts = bin_counts + tmp_cnts;
    for j = 1:length(y_store)
        bin_val(idx(j)) = bin_val(idx(j)) + y_store(j);
    end
    
    prior_idx = floor(x_store * prior_num) + 1;
    for j = 1:length(idx)
        prior_w(prior_idx(j)) = prior_w(prior_idx(j)) + 1;
        prior_cnt(prior_idx(j)) = prior_cnt(prior_idx(j)) + bin_counts(idx(j));
    end
    prior_p = prior_cnt ./ max(prior_w, 1e-8);
    prior_p = prior_p - min(prior_p);
    prior_p = exp(-prior_p);
    
    err = bin_val ./ bin_counts - ref;
    
    if iter_disp > disp_gap
        figure(1); clf;
        subplot(1,3,1);
        yyaxis left;
        plot(bin_centers, bin_val ./ bin_counts, bin_centers, ref);
        set(gca, 'ylim', [0, yr/beta*1.05]);
        yyaxis right;
        plot(bin_centers, err);
        set(gca, 'ylim', [-1,1]*0.1);
        title(sprintf('Iter: %d, total: %d', iter_num, sum(bin_counts)));
        subplot(1,3,2);
        plot(1/prior_num/2:1/prior_num:1, prior_p, '-o');
%         plot(bin_centers, x_counts, '-o');
%         set(gca, 'ylim', [0, 1]);
        subplot(1,3,3);
        plot(bin_centers, bin_counts);
        drawnow();
        iter_disp = iter_disp - disp_gap;
    end
end

figure(1); clf;
subplot(1,3,1);
yyaxis left;
plot(bin_centers, bin_val ./ bin_counts, bin_centers, ref);
set(gca, 'ylim', [0, yr/beta*1.05]);
yyaxis right;
plot(bin_centers, err);
set(gca, 'ylim', [-1,1]*0.1);
title(sprintf('Iter: %d, total: %d', iter_num, sum(bin_counts)));
subplot(1,3,2);
plot(1/prior_num/2:1/prior_num:1, prior_p, '-o');
% plot(bin_centers, x_counts, '-o');
% set(gca, 'ylim', [0, 1]);
subplot(1,3,3);
plot(bin_centers, bin_counts);
