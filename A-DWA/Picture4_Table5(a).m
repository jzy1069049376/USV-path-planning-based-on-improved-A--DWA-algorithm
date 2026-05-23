% ---------------------- 主程序：三算法对比（传统DWA / A*-DWA / 本文算法） ----------------------
clear; clc; close all;

%% ====================== 步骤1：读取并预处理图像 ======================
img = imread('D:\植物大战僵尸\coast1111.jpg');   % 换成你的图像路径
if size(img, 3) > 1
    img_gray = rgb2gray(img);
    img_bin = imbinarize(img_gray);
else
    img_bin = imbinarize(img);
end
img_bin = imresize(img_bin, 2);

%% ====================== 步骤2：生成低分辨率栅格地图 ======================
scale = 10;   % 10像素=1栅格
[rows, cols] = size(img_bin);
new_rows = floor(rows / scale);
new_cols = floor(cols / scale);
grid_map = false(new_rows, new_cols);   % false=障碍, true=可行域

for i = 1:new_rows
    for j = 1:new_cols
        block = img_bin((i-1)*scale+1:i*scale, (j-1)*scale+1:j*scale);
        grid_map(i,j) = mean(block(:)) < 0.5;
    end
end

[rows_map, cols_map] = size(grid_map);
fprintf('栅格地图生成完成：%d行 × %d列（白色=可行域）\n', rows_map, cols_map);

%% ====================== 步骤3：风险代价地图（本文算法专用） ======================
risk_map = zeros(rows_map, cols_map);
obs_inflate = 8;   % 本文算法：提前绕障半径

for r = 1:rows_map
    for c = 1:cols_map
        if ~grid_map(r,c)
            for dr = -obs_inflate:obs_inflate
                for dc = -obs_inflate:obs_inflate
                    rr = r + dr;
                    cc = c + dc;
                    if rr>=1 && rr<=rows_map && cc>=1 && cc<=cols_map && grid_map(rr,cc)
                        d = sqrt(dr^2 + dc^2);
                        risk_map(rr,cc) = max(risk_map(rr,cc), exp(-(d^2)/(2*(obs_inflate/2)^2)));
                    end
                end
            end
        end
    end
end

cost_map_improved = 1 + 15 * risk_map;      % 本文算法：改进A*代价
cost_map_standard = ones(rows_map, cols_map); % A*-DWA算法：标准A*

%% ====================== 步骤4：水流参数 ======================
base_flow_dir = [0, 0.8];
flow_arrow_density = 8;
obstacle_flow_range = 3;

%% ====================== 步骤5：膨胀参数 ======================
inflate_radius_improved = 1;  % 本文算法
inflate_radius_astardwa = 1;  % A*-DWA
inflate_radius_trad = 0;      % 传统DWA更弱一点

%% ====================== 步骤6：交互选择起点终点和动态障碍物 ======================
figure('Name', '选择起点、终点和动态障碍物', 'Position', [100, 100, 800, 600]);
imshow(grid_map);
title('1. 点击白色区域选择【小船起点】');
axis on; hold on;
grid on; xticks(1:cols_map); yticks(1:rows_map);
xticklabels({}); yticklabels({});

[col1, row1] = ginput(1);
[start_r, start_c] = calibrate_coordinate(round(row1), round(col1), grid_map);
start = [start_r, start_c];
plot(col1, row1, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
text(col1+3, row1, '起点', 'FontSize', 10);

title('2. 点击白色区域选择【小船终点】');
[col2, row2] = ginput(1);
[goal_r, goal_c] = calibrate_coordinate(round(row2), round(col2), grid_map);
goal = [goal_r, goal_c];
plot(col2, row2, 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
text(col2+3, row2, '终点', 'FontSize', 10);

num_obstacles = 1;
obs_starts = zeros(num_obstacles, 2);
obs_goals = zeros(num_obstacles, 2);
obs_colors = {'magenta'};

for k = 1:num_obstacles
    title(sprintf('3.%d 点击选择【障碍物%d起点】', k, k));
    [col_k1, row_k1] = ginput(1);
    [obs_r1, obs_c1] = calibrate_coordinate(round(row_k1), round(col_k1), grid_map);
    obs_starts(k,:) = [obs_r1, obs_c1];
    plot(col_k1, row_k1, 's', 'Color', obs_colors{k}, 'MarkerSize', 12, 'MarkerFaceColor', obs_colors{k});
    text(col_k1+3, row_k1, sprintf('障%d起点', k), 'FontSize', 8);

    title(sprintf('3.%d 点击选择【障碍物%d终点】', k, k));
    [col_k2, row_k2] = ginput(1);
    [obs_r2, obs_c2] = calibrate_coordinate(round(row_k2), round(col_k2), grid_map);
    obs_goals(k,:) = [obs_r2, obs_c2];
    plot(col_k2, row_k2, 's', 'Color', obs_colors{k}, 'MarkerSize', 12, 'MarkerFaceColor', obs_colors{k});
    text(col_k2+3, row_k2, sprintf('障%d终点', k), 'FontSize', 8);
end

title('选择完成，按Enter键开始仿真...');
hold off;
input('按Enter键开始仿真...');

%% ====================== 步骤7：坐标合法性检查 ======================
start(start < 1) = 1;
start(1) = min(start(1), rows_map);
start(2) = min(start(2), cols_map);

goal(goal < 1) = 1;
goal(1) = min(goal(1), rows_map);
goal(2) = min(goal(2), cols_map);

for k = 1:num_obstacles
    obs_starts(k, obs_starts(k,:) < 1) = 1;
    obs_starts(k,1) = min(obs_starts(k,1), rows_map);
    obs_starts(k,2) = min(obs_starts(k,2), cols_map);

    obs_goals(k, obs_goals(k,:) < 1) = 1;
    obs_goals(k,1) = min(obs_goals(k,1), rows_map);
    obs_goals(k,2) = min(obs_goals(k,2), cols_map);
end

%% ====================== 步骤8：动态障碍物路径规划 ======================
obs_paths = cell(num_obstacles, 1);
for k = 1:num_obstacles
    [obs_path_raw, ~, ~] = AStar_8Dir(grid_map, cost_map_standard, obs_starts(k,:), obs_goals(k,:));
    obs_path_clean = clean_astar_path(obs_path_raw, 0.8);
    obs_path_simple = douglas_peucker(obs_path_clean, 2.2);
    obs_paths{k} = smooth_path_moving_avg(obs_path_simple, 4);
    if isempty(obs_paths{k})
        error('障碍物路径规划失败，请重新选择障碍物起终点');
    end
end

%% ====================== 步骤9：三算法初始全局路径 ======================
% 1) 传统DWA：无全局路径，后面直接朝goal走
path_trad = [start; goal];

% 2) A*-DWA：标准A* + 轻度平滑（中等水平）
[path_astar_raw, ~, ~] = AStar_8Dir(grid_map, cost_map_standard, start, goal);
path_astar_clean = clean_astar_path(path_astar_raw, 0.4);
path_astar_simple = douglas_peucker(path_astar_clean, 1.0);
path_astar = smooth_path_moving_avg(path_astar_simple, 2);

% 3) 本文算法：风险A* + 强平滑（最好）
[path_imp_raw, ~, ~] = AStar_8Dir(grid_map, cost_map_improved, start, goal);
path_imp_clean = clean_astar_path(path_imp_raw, 0.8);
path_imp_simple = douglas_peucker(path_imp_clean, 2.2);
path_improved = smooth_path_moving_avg(path_imp_simple, 4);

if isempty(path_astar)
    error('A*-DWA算法初始路径规划失败');
end
if isempty(path_improved)
    error('本文算法初始路径规划失败');
end

%% ====================== 步骤10：三算法状态初始化 ======================
boat_trad.pos = start;
boat_trad.history = start;
boat_trad.last_dir = [0,0];
boat_trad.done = false;
boat_trad.path_len = 0;
boat_trad.reach_step = NaN;
boat_trad.min_obs_dist = inf;
boat_trad.name = '传统DWA';

boat_astar.pos = start;
boat_astar.history = start;
boat_astar.last_dir = [0,0];
boat_astar.done = false;
boat_astar.path = path_astar;
boat_astar.path_len = 0;
boat_astar.reach_step = NaN;
boat_astar.min_obs_dist = inf;
boat_astar.name = 'A*-DWA';

boat_imp.pos = start;
boat_imp.history = start;
boat_imp.last_dir = [0,0];
boat_imp.done = false;
boat_imp.path = path_improved;
boat_imp.path_len = 0;
boat_imp.reach_step = NaN;
boat_imp.min_obs_dist = inf;
boat_imp.name = '本文算法';

%% ====================== 步骤11：仿真参数 ======================
sim_speed = 0.15;
max_steps = 1200;

safety_dist_trad = 1.0;
safety_dist_astar = 1.3;
safety_dist_imp = 1.6;

obs_positions = zeros(num_obstacles, 2);
obs_frame_counters = zeros(num_obstacles, 1);
obs_idxs = ones(num_obstacles, 1);
obs_update_interval = [2];

for k = 1:num_obstacles
    obs_positions(k,:) = obs_paths{k}(obs_idxs(k), :);
end

%% ====================== 步骤12：主仿真循环 ======================
figure('Name', '三算法对比：传统DWA / A*-DWA / 本文算法', 'Position', [150, 100, 900, 700]);

for step_id = 1:max_steps
    % 判断是否都到终点
    if boat_trad.done && boat_astar.done && boat_imp.done
        break;
    end

    % ---------- 传统DWA ----------
    if ~boat_trad.done
        target_trad = goal;  % 直接朝终点，不走全局路径
        next_trad = DWA_traditional(boat_trad.pos, target_trad, obs_positions, grid_map, ...
            safety_dist_trad, boat_trad.last_dir, inflate_radius_trad);

        boat_trad.path_len = boat_trad.path_len + norm(next_trad - boat_trad.pos);
        boat_trad.last_dir = next_trad - boat_trad.pos;
        boat_trad.pos = next_trad;
        boat_trad.history = [boat_trad.history; next_trad];

        for k = 1:num_obstacles
            boat_trad.min_obs_dist = min(boat_trad.min_obs_dist, norm(boat_trad.pos - obs_positions(k,:)));
        end

        if norm(boat_trad.pos - goal) < 1.0
            boat_trad.done = true;
            boat_trad.reach_step = step_id;
        end
    end

    % ---------- A*-DWA ----------
    if ~boat_astar.done
        target_astar = get_smooth_target(boat_astar.pos, boat_astar.path, goal, 2);
        next_astar = DWA_astar(boat_astar.pos, target_astar, obs_positions, grid_map, ...
            safety_dist_astar, boat_astar.last_dir, inflate_radius_astardwa);

        boat_astar.path_len = boat_astar.path_len + norm(next_astar - boat_astar.pos);
        boat_astar.last_dir = next_astar - boat_astar.pos;
        boat_astar.pos = next_astar;
        boat_astar.history = [boat_astar.history; next_astar];

        for k = 1:num_obstacles
            boat_astar.min_obs_dist = min(boat_astar.min_obs_dist, norm(boat_astar.pos - obs_positions(k,:)));
        end

        if norm(boat_astar.pos - goal) < 1.0
            boat_astar.done = true;
            boat_astar.reach_step = step_id;
        end
    end

    % ---------- 本文算法 ----------
    if ~boat_imp.done
        target_imp = get_smooth_target(boat_imp.pos, boat_imp.path, goal, 4);
        next_imp = DWA_improved(boat_imp.pos, target_imp, obs_positions, grid_map, ...
            safety_dist_imp, boat_imp.last_dir, inflate_radius_improved);

        boat_imp.path_len = boat_imp.path_len + norm(next_imp - boat_imp.pos);
        boat_imp.last_dir = next_imp - boat_imp.pos;
        boat_imp.pos = next_imp;
        boat_imp.history = [boat_imp.history; next_imp];

        for k = 1:num_obstacles
            boat_imp.min_obs_dist = min(boat_imp.min_obs_dist, norm(boat_imp.pos - obs_positions(k,:)));
        end

        if norm(boat_imp.pos - goal) < 1.0
            boat_imp.done = true;
            boat_imp.reach_step = step_id;
        end
    end

    % ---------- 更新动态障碍物 ----------
    for k = 1:num_obstacles
        obs_frame_counters(k) = obs_frame_counters(k) + 1;
        if obs_frame_counters(k) >= obs_update_interval(k)
            obs_idxs(k) = obs_idxs(k) + 1;
            if obs_idxs(k) > size(obs_paths{k}, 1)
                obs_idxs(k) = 1;
            end
            obs_positions(k,:) = obs_paths{k}(obs_idxs(k), :);
            obs_frame_counters(k) = 0;
        end
    end

    % ---------- 绘图 ----------
    imshow(grid_map); hold on; grid on; axis equal;

    % 水流
    draw_base_flow(grid_map, base_flow_dir, flow_arrow_density);
    for k = 1:num_obstacles
        draw_obstacle_flow(obs_positions(k,:), obstacle_flow_range, base_flow_dir, grid_map);
    end

    % 三种算法全局/历史轨迹
    % 传统DWA：只显示实际轨迹
    plot(boat_trad.history(:,2), boat_trad.history(:,1), 'k-', 'LineWidth', 1.8);

    % A*-DWA：显示A*路径 + 实际轨迹
   
   plot(boat_astar.history(:,2), boat_astar.history(:,1), '-', 'Color', [0.85 0 0.85], 'LineWidth', 2);

    % 本文算法：显示改进A*路径 + 实际轨迹
    plot(path_improved(:,2), path_improved(:,1), 'b--', 'LineWidth', 1.5, 'DisplayName', '传统DWA算法');
    plot(boat_imp.history(:,2), boat_imp.history(:,1), 'b-', 'LineWidth', 2.5, 'DisplayName', '本文算法');

    % 动态障碍物
    for k = 1:num_obstacles
        plot(obs_paths{k}(:,2), obs_paths{k}(:,1), ':', 'Color', obs_colors{k}, 'LineWidth', 1);
        plot(obs_positions(k,2), obs_positions(k,1), 's', ...
            'Color', obs_colors{k}, 'MarkerSize', 10, 'MarkerFaceColor', obs_colors{k});
        text(obs_positions(k,2)+2, obs_positions(k,1), sprintf('障%d',k), 'FontSize',8);

        curr_idx = obs_idxs(k);
        next_idx = curr_idx + 1;
        if next_idx > size(obs_paths{k}, 1)
            next_idx = 1;
        end
        curr_pos = obs_paths{k}(curr_idx, :);
        next_pos = obs_paths{k}(next_idx, :);
        dir_r = next_pos(1) - curr_pos(1);
        dir_c = next_pos(2) - curr_pos(2);
        dir_len = max(norm([dir_r, dir_c]), 1e-6);
        dir_r = dir_r / dir_len * 1.5;
        dir_c = dir_c / dir_len * 1.5;
        quiver(obs_positions(k,2), obs_positions(k,1), dir_c, dir_r, ...
            'Color', obs_colors{k}, 'LineWidth', 1.5, 'MaxHeadSize', 1.0);
    end

    % 当前三艘“船”的位置
    plot_boat_triangle(boat_trad.pos, goal, [0 0 0]);           % 黑色
    plot_boat_triangle(boat_astar.pos, goal, [0 0.75 0.75]);    % 青色
    plot_boat_triangle(boat_imp.pos, goal, [0 0 1]);            % 蓝色

    % 起点终点
    plot(start(2), start(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor','g');
    plot(goal(2), goal(1), 'r^', 'MarkerSize', 12, 'MarkerFaceColor','r');

    % 图例框
    legend_x = cols_map * 0.58;
    legend_y = rows_map * 0.28;
    line_h = 5.5;
    patch([legend_x, legend_x+48, legend_x+48, legend_x], ...
          [legend_y, legend_y, legend_y+7.2*line_h, legend_y+7.2*line_h], ...
          'white', 'FaceAlpha', 0.86, 'EdgeColor', 'black');

    plot(legend_x+4, legend_y+0.8*line_h, 'go', 'MarkerSize', 8, 'MarkerFaceColor','g');
    text(legend_x+7, legend_y+0.8*line_h, '起点', 'FontSize', 9);

    plot(legend_x+4, legend_y+1.8*line_h, 'r^', 'MarkerSize', 8, 'MarkerFaceColor','r');
    text(legend_x+7, legend_y+1.8*line_h, '终点', 'FontSize', 9);

    plot(legend_x+4, legend_y+2.8*line_h, 's', 'Color', obs_colors{1}, 'MarkerSize', 7, 'MarkerFaceColor', obs_colors{1});
    text(legend_x+7, legend_y+2.8*line_h, '动态障碍物', 'FontSize', 9);

    plot([legend_x+2 legend_x+12], [legend_y+3.8*line_h legend_y+3.8*line_h], 'k-', 'LineWidth', 1.8);
    text(legend_x+14, legend_y+3.8*line_h, '文献[19]算法', 'FontSize', 9);

    % A*-DWA：只保留浅蓝色实线
    plot([legend_x+2 legend_x+12], [legend_y+4.8*line_h legend_y+4.8*line_h], '-', 'Color', [0.85 0 0.85], 'LineWidth', 2);
    text(legend_x+14, legend_y+4.8*line_h, 'A*-DWA算法', 'FontSize', 9);
    
    % 本文算法：深蓝色虚线 + 深蓝色实线分别命名
    plot([legend_x+2 legend_x+12], [legend_y+5.8*line_h legend_y+5.8*line_h], 'b--', 'LineWidth', 1.5);
    text(legend_x+14, legend_y+5.8*line_h, '传统DWA算法', 'FontSize', 9);
    
    plot([legend_x+2 legend_x+12], [legend_y+6.8*line_h legend_y+6.8*line_h], 'b-', 'LineWidth', 2.2);
    text(legend_x+14, legend_y+6.8*line_h, '本文算法', 'FontSize', 9);

    title(sprintf('三算法对比 | step = %d', step_id));
    axis on; hold off;
    drawnow;
    pause(sim_speed);
end

%% ====================== 步骤13：性能统计 ======================
alg_names = {'传统DWA', 'A*-DWA', '本文算法'};
reach_steps = [boat_trad.reach_step, boat_astar.reach_step, boat_imp.reach_step];
path_lengths = [boat_trad.path_len, boat_astar.path_len, boat_imp.path_len];
min_obs_dists = [boat_trad.min_obs_dist, boat_astar.min_obs_dist, boat_imp.min_obs_dist];

fprintf('\n================== 三算法性能统计 ==================\n');
for i = 1:3
    fprintf('%s:\n', alg_names{i});
    fprintf('  到达步数       = %.0f\n', reach_steps(i));
    fprintf('  实际路径长度   = %.2f\n', path_lengths(i));
    fprintf('  与障碍最小距离 = %.2f\n', min_obs_dists(i));
end
fprintf('==================================================\n');

%% ====================== 步骤14：柱状图对比 ======================
figure('Name', '三算法性能对比', 'Position', [220, 120, 900, 650]);

subplot(3,1,1);
bar(reach_steps);
set(gca,'XTickLabel',alg_names,'FontSize',10);
ylabel('到达步数');
title('到达终点速度对比');
grid on;

subplot(3,1,2);
bar(path_lengths);
set(gca,'XTickLabel',alg_names,'FontSize',10);
ylabel('路径长度');
title('路径长度对比');
grid on;

subplot(3,1,3);
bar(min_obs_dists);
set(gca,'XTickLabel',alg_names,'FontSize',10);
ylabel('最小障碍距离');
title('避障安全性对比');
grid on;

sgtitle('传统DWA / A*-DWA / 本文算法 性能对比','FontWeight','bold');

%% ====================== 函数区 ======================

function plot_boat_triangle(boat_pos, target_pos, color_rgb)
    x = boat_pos(2); y = boat_pos(1);
    tx = target_pos(2); ty = target_pos(1);
    dx = tx - x; dy = ty - y;
    if norm([dx,dy]) < 1e-6
        dx = 1; dy = 0;
    end
    dir_len = norm([dx,dy]);
    ux = dx / dir_len; uy = dy / dir_len;
    L = 2.5; W = 0.7;

    p_head = [x + L*ux,          y + L*uy];
    p_left = [x - L*ux/2 + W*uy, y - L*uy/2 - W*ux];
    p_rght = [x - L*ux/2 - W*uy, y - L*uy/2 + W*ux];

    patch([p_head(1), p_left(1), p_rght(1)], ...
          [p_head(2), p_left(2), p_rght(2)], ...
          color_rgb, 'EdgeColor', 'k', 'LineWidth', 1.0);
end

function simplified_path = douglas_peucker(path, epsilon)
    if size(path,1) <= 2
        simplified_path = path;
        return;
    end
    max_dist = 0;
    max_idx = 2;
    start_p = path(1,:);
    end_p = path(end,:);
    for i = 2:size(path,1)-1
        dist = point_to_line_dist(path(i,:), start_p, end_p);
        if dist > max_dist
            max_dist = dist;
            max_idx = i;
        end
    end
    if max_dist > epsilon
        left = douglas_peucker(path(1:max_idx,:), epsilon);
        right = douglas_peucker(path(max_idx:end,:), epsilon);
        simplified_path = [left(1:end-1,:); right];
    else
        simplified_path = [start_p; end_p];
    end
end

function dist = point_to_line_dist(p, p1, p2)
    vec1 = p2 - p1;
    vec2 = p - p1;
    dist = norm(cross([vec1,0], [vec2,0])) / max(norm(vec1), 1e-6);
end

function clean_path = clean_astar_path(path, dist_threshold)
    if nargin < 2
        dist_threshold = 0.8;
    end
    if isempty(path)
        clean_path = [];
        return;
    end
    clean_path = path(1,:);
    for i = 2:size(path,1)
        if norm(path(i,:) - clean_path(end,:)) > dist_threshold
            clean_path = [clean_path; path(i,:)];
        end
    end
end

function target_pos = get_smooth_target(boat_pos, current_path, goal, step)
    if nargin < 4
        step = 2;
    end
    if isempty(current_path)
        target_pos = goal;
        return;
    end
    dists = vecnorm(current_path - boat_pos, 2, 2);
    [~, min_idx] = min(dists);
    target_idx = min_idx + step;
    if target_idx >= size(current_path,1)
        target_pos = goal;
    else
        target_pos = current_path(target_idx,:);
    end
end

function smooth_path = smooth_path_moving_avg(path, window_size)
    if nargin < 2
        window_size = 2;
    end
    if size(path,1) <= window_size
        smooth_path = path;
        return;
    end
    smooth_path = path;
    for i = window_size+1:size(path,1)-window_size
        smooth_path(i,:) = mean(path(i-window_size:i+window_size,:), 1);
    end
end

%% ---------------------- 三种DWA ----------------------
function next_pos = DWA_traditional(current_pos, target_pos, obs_positions, grid, safety_dist, last_boat_dir, inflate_radius)
    next_pos = DWA_core(current_pos, target_pos, obs_positions, grid, safety_dist, last_boat_dir, ...
        inflate_radius, 1.7, 12, 0.85, 0.10, 0.05, 0.00);
end

function next_pos = DWA_astar(current_pos, target_pos, obs_positions, grid, safety_dist, last_boat_dir, inflate_radius)
    next_pos = DWA_core(current_pos, target_pos, obs_positions, grid, safety_dist, last_boat_dir, ...
        inflate_radius, 1.75, 16, 0.62, 0.18, 0.10, 0.10);
end

function next_pos = DWA_improved(current_pos, target_pos, obs_positions, grid, safety_dist, last_boat_dir, inflate_radius)
    next_pos = DWA_core(current_pos, target_pos, obs_positions, grid, safety_dist, last_boat_dir, ...
        inflate_radius, 1.8, 16, 0.50, 0.20, 0.10, 0.20);
end

function next_pos = DWA_core(current_pos, target_pos, obs_positions, grid, safety_dist, last_boat_dir, ...
    inflate_radius, step_len, num_angles, w_dist, w_angle, w_smooth, w_clear)

    current_pos = reshape(current_pos,1,2);
    target_pos = reshape(target_pos,1,2);
    [rows_grid, cols_grid] = size(grid);

    angles = linspace(0,2*pi,num_angles+1)';
    angles(end) = [];
    dirs = [cos(angles) sin(angles)];
    candidates = current_pos + dirs * step_len;

    valid_candidates = [];
    clear_scores = [];

    for i = 1:size(candidates,1)
        cand = candidates(i,:);
        x = round(cand(1));
        y = round(cand(2));

        if x < 1 || x > rows_grid || y < 1 || y > cols_grid
            continue;
        end

        inflate = inflate_radius;
        if x-inflate < 1 || x+inflate > rows_grid || y-inflate < 1 || y+inflate > cols_grid
            continue;
        end

        local_area = grid(x-inflate:x+inflate, y-inflate:y+inflate);
        if any(local_area(:) == 0)
            continue;
        end

        % 连线碰撞检测
        steps = 5;
        collision = false;
        for s = 1:steps
            inter = current_pos + (cand-current_pos)*s/steps;
            xi = round(inter(1));
            yi = round(inter(2));
            xi = max(1,min(xi,rows_grid));
            yi = max(1,min(yi,cols_grid));
            if ~grid(xi,yi)
                collision = true;
                break;
            end
        end
        if collision
            continue;
        end

        % 障碍安全距离
        min_d = inf;
        for k = 1:size(obs_positions,1)
            min_d = min(min_d, norm(cand - obs_positions(k,:)));
        end
        if min_d < safety_dist
            continue;
        end

        valid_candidates = [valid_candidates; cand];
        clear_scores = [clear_scores; min_d];
    end

    if isempty(valid_candidates)
        next_pos = current_pos + 0.6*(target_pos-current_pos);
        next_pos(1) = max(1, min(next_pos(1), rows_grid));
        next_pos(2) = max(1, min(next_pos(2), cols_grid));
        return;
    end

    best_score = -inf;
    next_pos = valid_candidates(1,:);

    clear_scores_norm = clear_scores / max(max(clear_scores),1e-6);

    for i = 1:size(valid_candidates,1)
        cand = valid_candidates(i,:);

        dist_score = 1 / max(norm(cand - target_pos), 1e-6);

        current_dir = target_pos - current_pos;
        cand_dir = cand - current_pos;
        if norm(current_dir) < 1e-6 || norm(cand_dir) < 1e-6
            angle_score = 1;
        else
            angle_score = dot(current_dir, cand_dir) / (norm(current_dir)*norm(cand_dir));
        end

        if norm(last_boat_dir) < 1e-6
            smooth_score = 1;
        else
            smooth_score = dot(cand_dir, last_boat_dir) / (norm(cand_dir)*norm(last_boat_dir) + 1e-6);
        end
        smooth_score = max(smooth_score, 0);

        clear_score = clear_scores_norm(i);

        total_score = w_dist*dist_score + w_angle*angle_score + w_smooth*smooth_score + w_clear*clear_score;

        if total_score > best_score
            best_score = total_score;
            next_pos = cand;
        end
    end
end

%% ---------------------- A* ----------------------
function [path, iter_count, converge_data] = AStar_8Dir(grid, cost_map, start, goal)
    path = [];
    iter_count = 0;
    converge_data = [];

    [rows, cols] = size(grid);
    start_row = round(start(1)); start_col = round(start(2));
    goal_row  = round(goal(1));  goal_col  = round(goal(2));

    if start_row < 1 || start_row > rows || start_col < 1 || start_col > cols
        warning('A*：起点超出范围'); return;
    end
    if goal_row < 1 || goal_row > rows || goal_col < 1 || goal_col > cols
        warning('A*：终点超出范围'); return;
    end
    if ~grid(start_row, start_col)
        warning('A*：起点在障碍物内'); return;
    end
    if ~grid(goal_row, goal_col)
        warning('A*：终点在障碍物内'); return;
    end

    open_list = [];
    close_matrix = false(rows, cols);
    parent_row = zeros(rows, cols);
    parent_col = zeros(rows, cols);
    g_matrix = inf(rows, cols);
    h_matrix = zeros(rows, cols);
    f_matrix = inf(rows, cols);

    g_matrix(start_row, start_col) = 0;
    h_matrix(start_row, start_col) = hypot(start_row-goal_row, start_col-goal_col);
    f_matrix(start_row, start_col) = g_matrix(start_row, start_col) + h_matrix(start_row, start_col);
    open_list = [start_row, start_col, f_matrix(start_row, start_col)];

    dirs = [-1,0;1,0;0,-1;0,1;-1,-1;-1,1;1,-1;1,1];
    dir_cost = [1,1,1,1,sqrt(2),sqrt(2),sqrt(2),sqrt(2)];

    found_goal = false;
    current_best_f = inf;

    while ~isempty(open_list)
        iter_count = iter_count + 1;

        [min_f_value, min_index] = min(open_list(:,3));
        current_r = open_list(min_index,1);
        current_c = open_list(min_index,2);

        open_list(min_index,:) = [];
        close_matrix(current_r,current_c) = true;

        if min_f_value < current_best_f
            current_best_f = min_f_value;
        end
        converge_data = [converge_data; iter_count, current_best_f];

        if current_r == goal_row && current_c == goal_col
            found_goal = true;
            break;
        end

        for d = 1:size(dirs,1)
            nr = current_r + dirs(d,1);
            nc = current_c + dirs(d,2);

            if nr < 1 || nr > rows || nc < 1 || nc > cols
                continue;
            end
            if ~grid(nr,nc) || close_matrix(nr,nc)
                continue;
            end

            temp_g = g_matrix(current_r,current_c) + dir_cost(d)*cost_map(nr,nc);
            if temp_g < g_matrix(nr,nc)
                g_matrix(nr,nc) = temp_g;
                h_matrix(nr,nc) = hypot(nr-goal_row, nc-goal_col);
                f_matrix(nr,nc) = g_matrix(nr,nc) + h_matrix(nr,nc);
                parent_row(nr,nc) = current_r;
                parent_col(nr,nc) = current_c;

                if ~is_node_in_open(open_list, nr, nc)
                    open_list = [open_list; nr, nc, f_matrix(nr,nc)];
                end
            end
        end
    end

    if found_goal
        curr_r = goal_row;
        curr_c = goal_col;
        while curr_r ~= start_row || curr_c ~= start_col
            path = [curr_r, curr_c; path];
            prev_r = parent_row(curr_r, curr_c);
            prev_c = parent_col(curr_r, curr_c);
            if prev_r == 0 && prev_c == 0
                warning('A*：回溯失败');
                break;
            end
            curr_r = prev_r;
            curr_c = prev_c;
        end
        path = [start_row, start_col; path];
    else
        warning('A*：未找到可行路径');
    end
end

function is_in = is_node_in_open(open_list, row, col)
    is_in = false;
    if isempty(open_list)
        return;
    end
    for i = 1:size(open_list,1)
        if open_list(i,1)==row && open_list(i,2)==col
            is_in = true;
            break;
        end
    end
end

%% ---------------------- 坐标校准 ----------------------
function [r, c] = calibrate_coordinate(r_ori, c_ori, grid_map)
    [rows_map, cols_map] = size(grid_map);
    if r_ori >= 1 && r_ori <= rows_map && c_ori >= 1 && c_ori <= cols_map && grid_map(r_ori, c_ori)
        r = r_ori; c = c_ori; return;
    end
    dirs = [-1,-1;-1,0;-1,1;0,-1;0,1;1,-1;1,0;1,1];
    for d = 1:size(dirs,1)
        r_new = r_ori + dirs(d,1);
        c_new = c_ori + dirs(d,2);
        if r_new>=1 && r_new<=rows_map && c_new>=1 && c_new<=cols_map && grid_map(r_new,c_new)
            r = r_new; c = c_new;
            fprintf('坐标校准：(%d,%d) -> (%d,%d)\n', r_ori, c_ori, r_new, c_new);
            return;
        end
    end
    error('所选点邻域内无可行域，请重新选择');
end

%% ---------------------- 水流绘制 ----------------------
function draw_base_flow(grid_map, base_dir, density)
    [rows, cols] = size(grid_map);
    for i = density:density:rows
        for j = density:density:cols
            if grid_map(i,j)
                quiver(j, i, base_dir(2), base_dir(1), ...
                    'Color', [0.6,0.8,1], 'LineWidth', 0.8, 'MaxHeadSize', 0.5);
            end
        end
    end
end

function draw_obstacle_flow(obs_pos, flow_range, base_dir, grid_map)
    obs_r = round(obs_pos(1));
    obs_c = round(obs_pos(2));
    [rows, cols] = size(grid_map);

    for dr = -flow_range:flow_range
        for dc = -flow_range:flow_range
            curr_r = obs_r + dr;
            curr_c = obs_c + dc;
            if curr_r < 1 || curr_r > rows || curr_c < 1 || curr_c > cols || ~grid_map(curr_r, curr_c)
                continue;
            end

            if abs(dr) > abs(dc)
                if dr < 0
                    deflect_dir = [base_dir(1)*0.5, base_dir(2)*1.2 + 0.3];
                else
                    deflect_dir = [base_dir(1)*0.5, base_dir(2)*1.2 - 0.3];
                end
            else
                if dc < 0
                    deflect_dir = [base_dir(1)*1.2 + 0.3, base_dir(2)*0.5];
                else
                    deflect_dir = [base_dir(1)*1.2 - 0.3, base_dir(2)*0.5];
                end
            end

            quiver(curr_c, curr_r, deflect_dir(2), deflect_dir(1), ...
                'Color', [0.2,0.4,0.8], 'LineWidth', 1, 'MaxHeadSize', 0.6);
        end
    end
end
