clear,clc;
% fname_tsp='Tsp_run';
% LKHdir=strcat(pwd,'/');
% TSPLIBdir=strcat(pwd,'/');
% lkh_cmd=[LKHdir, 'LKH', ' ', TSPLIBdir, fname_tsp, '.par'];
%  
% %status 为零表示命令已成功完成。MATLAB 将在 cmdout 中返回一个包含当前文件夹的字符向量
% [status, cmdout]=system(lkh_cmd,'-echo');
% 
% %%先用strfind函数设置好id和_iso的位置，
% %然后再根据这两个位置直接提取字符串中在这两个位置之间的字符串就是你所需要的数字
% %strfind(s1,s2)--or strfind(s1,pattern), 其意思在s1中搜索pattern
% 
% first=strfind(cmdout, 'Cost.min = ');
% last=strfind(cmdout, ', Cost.avg');
% minCost=str2num(cmdout(first+11:last-1));

% 读取 TSP 数据文件
filename = 'Tsp_data.tsp';
fileID = fopen(filename, 'r');

% 寻找NODE_COORD_SECTION标记
while true
    line = fgetl(fileID);
    if strcmp(line, 'NODE_COORD_SECTION')
        break;
    end
end

% 读取坐标数据
coordinates = fscanf(fileID, '%d %f %f %f', [4, Inf]);
fclose(fileID);

% 提取坐标
points = coordinates(2:4, :)';

% 读取解决方案文件
tour_file = 'output.txt';
fileID = fopen(tour_file, 'r');

% 寻找TOUR_SECTION标记
while true
    line = fgetl(fileID);
    if strcmp(line, 'TOUR_SECTION')
        break;
    end
end

% 读取解决方案
solution_order = fscanf(fileID, '%d', [1, Inf]);
fclose(fileID);

% 按照解决方案顺序重新排列坐标
ordered_points = zeros(length(solution_order), size(points, 2));

% 使用循环逐个构建 ordered_points
for i = 1:length(solution_order)
    index = solution_order(i);
    
    % 检查索引的有效性，可以根据需要进行处理
    if index < 1
        index = 1;
    elseif index > size(points, 1)
        index = size(points, 1);
    end
    
    % 将相应的坐标放入 ordered_points
    ordered_points(i, :) = points(index, :);
end

% % 绘制原始点
% figure;
% scatter3(points(:, 1), points(:, 2), points(:, 3), 'filled');
% title('Original Points');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

% 绘制按照解决方案顺序重新排列后的点
figure;
scatter3(ordered_points(:, 1), ordered_points(:, 2), ordered_points(:, 3), 'filled');
title('Ordered Points');
xlabel('X');
ylabel('Y');
zlabel('Z');

% 绘制按照解决方案顺序连接的路径
figure;
plot3(ordered_points(:, 1), ordered_points(:, 2), ordered_points(:, 3), 'r-', 'LineWidth', 2);
title('Ordered TSP Solution');
xlabel('X');
ylabel('Y');
zlabel('Z');