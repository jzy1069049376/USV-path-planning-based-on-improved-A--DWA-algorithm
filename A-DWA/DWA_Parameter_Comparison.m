%% alpha, beta, gamma, lambda 参数组合敏感性实验
clear; clc; close all;

%% 1. 参数组合
Group = {'组合一'; '组合二'; '组合三'; '组合四'; '组合五'};

alpha  = [0.30; 0.25; 0.20; 0.20; 0.30];
beta   = [0.25; 0.20; 0.20; 0.15; 0.15];
gamma  = [0.30; 0.35; 0.35; 0.45; 0.25];
lambda = [0.15; 0.20; 0.25; 0.20; 0.30];

%% 2. 不同参数组合下的实验结果
% 对应前面给你的 DWA 评价函数权重敏感性实验数据
SearchTime = [226.1; 229.4; 236.5; 247.1; 235.3];   % 搜索时间 / ms
PathLength = [99.9; 101.4; 105.6; 108.7; 106.2];     % 路径长度 / m
SafeDist   = [2.5; 3.0; 3.1; 3.8; 2.3];              % 安全距离 / m
Offset     = [6.3; 5.8; 5.4; 5.7; 5.2];              % 轨迹偏移量 / m

%% 3. 指标归一化
% 代价型指标：搜索时间、路径长度、轨迹偏移量，越小越好
T_norm = (SearchTime - min(SearchTime)) ./ (max(SearchTime) - min(SearchTime));
L_norm = (PathLength - min(PathLength)) ./ (max(PathLength) - min(PathLength));
E_norm = (Offset - min(Offset)) ./ (max(Offset) - min(Offset));

% 效益型指标：安全距离，越大越好
D_norm = (SafeDist - min(SafeDist)) ./ (max(SafeDist) - min(SafeDist));
D_cost = 1 - D_norm;

%% 4. 综合性能指标计算
% 权重可根据论文指标重要性适当调整
wT = 0.20;   % 搜索时间权重
wL = 0.25;   % 路径长度权重
wD = 0.25;   % 安全距离权重
wE = 0.30;   % 轨迹偏移量权重

J = wT*T_norm + wL*L_norm + wD*D_cost + wE*E_norm;

%% 5. 输出结果表
ResultTable = table(Group, alpha, beta, gamma, lambda, ...
    SearchTime, PathLength, SafeDist, Offset, J, ...
    'VariableNames', {'参数组合','alpha','beta','gamma','lambda', ...
    '搜索时间_ms','路径长度_m','安全距离_m','轨迹偏移量_m','综合性能指标'});

disp(ResultTable);

%% 6. 找出最优组合
[~, idx_best] = min(J);

fprintf('\n最优参数组合为：%s\n', Group{idx_best});
fprintf('alpha = %.2f, beta = %.2f, gamma = %.2f, lambda = %.2f\n', ...
    alpha(idx_best), beta(idx_best), gamma(idx_best), lambda(idx_best));
fprintf('综合性能指标 J = %.3f\n', J(idx_best));

%% 7. 绘制综合性能指标柱状图
figure('Color','w','Position',[300 200 720 420]);

bar(J, 0.55);
grid on;
box on;

set(gca, ...
    'XTick', 1:length(Group), ...
    'XTickLabel', Group, ...
    'FontName', '宋体', ...
    'FontSize', 12, ...
    'LineWidth', 1.0);

ylabel('综合性能指标', 'FontName','宋体', 'FontSize',13);
xlabel('参数组合', 'FontName','宋体', 'FontSize',13);

title('不同 \alpha、\beta、\gamma、\lambda 参数组合下的综合性能对比', ...
    'FontName','宋体', 'FontSize',13);

for i = 1:length(J)
    text(i, J(i)+0.02, sprintf('%.3f', J(i)), ...
        'HorizontalAlignment','center', ...
        'FontName','Times New Roman', ...
        'FontSize',11);
end

ylim([0, max(J)+0.15]);

%% 8. 绘制各项性能指标归一化对比图
figure('Color','w','Position',[300 200 780 460]);

Data = [SearchTime./max(SearchTime), ...
        PathLength./max(PathLength), ...
        SafeDist./max(SafeDist), ...
        Offset./max(Offset)];

bar(Data, 'grouped');
grid on;
box on;

set(gca, ...
    'XTick', 1:length(Group), ...
    'XTickLabel', Group, ...
    'FontName','宋体', ...
    'FontSize',12, ...
    'LineWidth',1.0);

legend({'搜索时间','路径长度','安全距离','轨迹偏移量'}, ...
    'Location','northoutside', ...
    'Orientation','horizontal', ...
    'FontName','宋体', ...
    'FontSize',11);

xlabel('参数组合', 'FontName','宋体', 'FontSize',13);
ylabel('归一化指标值', 'FontName','宋体', 'FontSize',13);

title('不同 DWA 权重组合下各项性能指标对比', ...
    'FontName','宋体', 'FontSize',13);