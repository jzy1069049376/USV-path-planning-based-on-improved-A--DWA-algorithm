% 步骤1：读取并预处理图像 + 放大原始图像
img = imread('D:\植物大战僵尸\coast1111.jpg');  % 读取图像
if size(img, 3) > 1
    img_gray = rgb2gray(img);  % 转为灰度图
    img_bin = imbinarize(img_gray);  % 二值化（黑白区分海陆）
else
    img_bin = imbinarize(img);  % 若已是单通道，直接二值化
end
% 放大原始图像（例如放大2倍）
img_bin = imresize(img_bin, 3);  % 调整缩放倍数可改变最终像素块大小

% 步骤2：降低分辨率（像素聚合）
scale = 10;  % 缩放因子（数值越大，像素块越大）
[rows, cols] = size(img_bin);
new_rows = floor(rows / scale);
new_cols = floor(cols / scale);
grid_lowres = false(new_rows, new_cols);  % 初始化低分辨率栅格

% 聚合像素：每个scale×scale的块按多数原则判断
for i = 1:new_rows
    for j = 1:new_cols
        block = img_bin((i-1)*scale+1:i*scale, (j-1)*scale+1:j*scale);
        grid_lowres(i,j) = mean(block(:)) > 0.5;  % 多数为陆地则保留（原逻辑不变）
    end
end

% 步骤3：平滑处理
se = strel('square', 3);  % 创建3×3结构元素
se_matrix = getnhood(se);  % 提取结构元素的数值矩阵
se_matrix = double(se_matrix);  % 转为double类型
grid_smoothed = imfilter(double(grid_lowres), se_matrix, 'replicate');  % 滤波
grid_map = ~(grid_smoothed > 0.5);  % 核心修改：取反逻辑值，实现黑白互换

% 步骤4：放大显示窗口
figure;
imshow(grid_map);
set(gcf, 'Position', [100, 100, 800, 600]);  % 调整窗口大小
title(['处理后栅格地图（分辨率：' num2str(scale) '米/像素，原始图放大2倍）']);

% 输出尺寸信息
[rows_final, cols_final] = size(grid_map);
fprintf('处理后栅格地图：%d行 × %d列，分辨率%d米/像素\n', rows_final, cols_final, scale);