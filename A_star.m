clear,clc, close all
global start_point end_point current_point ValueSet 
row = 50;
clm = 50; 
%%
%启发式搜索代价函数类型
% 1为欧式距离；2为曼哈顿距离；3为对角线距离；4为Dijkstra算法；5引入权重函数的欧式距离
HType = 1;  

W = 0.2; %权重系数
%% 初始化定义
% 将坐标存储在一个结构体中
NewPoint = @(a, b) struct('x', a, 'y', b); 
% 将x.y坐标转换为一个长为6的字符串
SetKeyStr = @(x, y) string(num2str([x, y], '%03d'));
% 起始点代价
InvalidKeyStr = "000000";
% 定义一个结构体，用于存储每一步的状态
NewSearchPoint = @(a, b) struct('key', SetKeyStr(a, b), 'x', a, 'y', b, 'G', 0, 'H', 0, 'F', 0, 'parent_key', InvalidKeyStr);
% 定义一个结构体，存储地图不同状态（用于染色）
ValueSet = struct('passable', 255, 'impassable', 0, 'openlist', 180, 'closelist', 120, 'start', 60, 'target', 30, 'current', 60, 'path', 60);
% map中的值设定
map = ones(row, clm) * ValueSet.passable;
%设置地图边界
[map(1, :), map(end, :), map(:, 1), map(:, end)] = deal(ValueSet.impassable);
%放置障碍物
load ev2;
ev=1-ev;
map = ev;
map(map==1)=255;% 给障碍物染色
%% 设置起始/结束点坐标

[start_point, end_point] = deal(NewPoint(2, 2), NewPoint(49, 49)); 
%%  设置初始状态

% 为起点和终点上色
[map(start_point.x, start_point.y), map(end_point.x, end_point.y)] = deal(ValueSet.start, ValueSet.target);
% 搜索点（周围8个）分别为dx dy 以及移动代价（实际为sqrt(dx*dx+dy*dy)）
SearchDxy = [-1, -1, sqrt(2); 0, -1, 1; 1, -1, sqrt(2); -1, 0, 1; 1, 0, 1; -1, 1, sqrt(2); 0, 1, 1; 1, 1, sqrt(2)];
% 将起始点加入open_list
open_list(1) = NewSearchPoint(start_point.x, start_point.y); % 初始状态设定
%计算预估代价
open_list(1).H = CalcH(start_point.x, start_point.y, end_point.x, end_point.y, HType,W);
%起点的当前代价为0
open_list(1).F = open_list.H + 0 ;
% 将open_list从结构体转化为table类型
open_list = struct2table(open_list); %待确定代价的点
close_list = []; %已确定代价的点
current_point = open_list; %当前点（设定为初始点）
figure % 打开画图

b_find_path = 0; % 是否发现路径，0为未发现
tic
%% 开始循环，直到open_list为空，退出循环

while ~isempty(open_list)% isempty如果open_list为空数组返回逻辑1
    % 遍历open_list，找到最小合代价F点
    index_min_open_list_F = SearchOptimalPoint(open_list, close_list, current_point, end_point);
    current_point = open_list(index_min_open_list_F, :); %最小代价F的点选中为当前点，进行后续open_list选取
    open_list(index_min_open_list_F, :) = []; %在open_list中将其删除
    close_list = [close_list; current_point]; %将其加入close_list
    map(current_point.x, current_point.y) = ValueSet.closelist; %将新加入的close_list点标记（染色）
    DrawMap(map); %绘图
    % 检查是否找到目标点，如果找到退出循环
    if current_point.x == end_point.x && current_point.y == end_point.y
        b_find_path = true;
        break;
    end
    % 检查当前点周围八个可移动点，将符合条件的点加入open_list中
    for search_dxy = SearchDxy'
        search_point = NewSearchPoint(current_point.x + search_dxy(1), current_point.y + search_dxy(2));
        key = SetKeyStr(search_point.x, search_point.y);
        %  如果它是不可抵达的或者它在 close list 中，忽略它
        if search_point.x <= 0 || search_point.y <= 0 || map(search_point.x, search_point.y) == ValueSet.impassable || map(search_point.x, search_point.y) == ValueSet.closelist
            continue;
        end
        search_point = struct2table(search_point);
        search_point.G = current_point.G + search_dxy(3); %移动代价
        search_point.H = CalcH(search_point.x, search_point.y, end_point.x, end_point.y, HType,W); %预估代价g
        search_point.F = search_point.G + search_point.H;% 计算得到的总代价
        search_point.parent_key = current_point.key;
        index_existed_in_openlist = find(open_list.key == key, 1); %判定当前open_list中是否存在该搜索点
        % 如果它不在 open list 中，把它加入 open list ，并且把当前方格设置为它的父节点，记录该方格的 F ， G 和 H 值
        if map(search_point.x, search_point.y) ~= ValueSet.openlist
            open_list = [open_list; search_point];
            map(search_point.x, search_point.y) = ValueSet.openlist; %将新加入的open_list点标记（染色）
        else % 如果它已经在 open list 中，检查这条路径 ( 即经由当前方格到达它那里 ) 是否更好，用 G 值作参考。更小的 G 值表示这是更好的路径。如果是这样，把它的父节点设置为当前方格，并重新计算它的 G 和 F 值。
            if search_point.G < open_list.G(index_existed_in_openlist)%若open_list中存在值的G值更大，表示由当前点到达该值更优，将原本储存点的信息替换为当前搜索点信息
                open_list(index_existed_in_openlist, :) = search_point; % 进行替换
            end
        end
    end
end
%% 如果发现路径，则按照parent信息回溯路径，导出结果

if b_find_path%找到路径
    path = close_list(end, :);
    path_step_count = 1;
    map(path.x(1), path.y(1)) = ValueSet.path;
    % 一直导出到InvalidKeyStr即起点
    while path.parent_key(path_step_count) ~= InvalidKeyStr
        path_step_count = path_step_count + 1;
        index_parent = find(close_list.key == path.parent_key(path_step_count - 1), 1);
        path(path_step_count, :) = close_list(index_parent, :);
        map(path.x(path_step_count), path.y(path_step_count)) = ValueSet.path;
    end
    DrawMap(map); %绘图
else
    disp('未找到有效路径！');
end
toc
disp(['openlist数量: ', num2str(height(open_list)), ', closelist数量: ', num2str(height(close_list)), ', 总代价: ', num2str(path.G(1), '%3.2f')])
hold on 
x = path.x;
y = path.y;
plot(y,x,'b');

%% 预估代价计算
function H = CalcH(x1, y1, x2, y2, type,W)

    dx = x2 - x1;
    dy = y2 - y1;
    switch type
        case 1
            H = sqrt(dx * dx + dy * dy); %欧式距离 
        case 2
            H = abs(dx) + abs(dy); %曼哈顿距离
        case 3
            h_diagonal = min(abs(dx), abs(dy));
            h_straight = abs(dx) + abs(dy);
            H = sqrt(2) * h_diagonal + (h_straight - 2 * h_diagonal); %对角线距离（切比雪夫距离）
        case 4
            H = 0; %Dijkstra算法
        case 5
            H = (dx * dx + dy * dy)*W;%改进的欧氏距离搜索函数
    end
end
%% 绘制map图
function DrawMap(map)
    global start_point end_point current_point ValueSet
    % 注意这里对map的操作只是为了显示效果，不会影响到主函数内的map，
    [map(start_point.x, start_point.y), map(end_point.x, end_point.y), map(current_point.x, current_point.y)] = deal(ValueSet.start, ValueSet.target, ValueSet.current);
     imagesc(map)% imagesc显示染色过的地图
     % 更改图片色调
     colormap hot
     % 显示网格
    grid on 
    set(gca,'xtick',[0.5:1:50.5])
    set(gca,'ytick',[0.5:1:50.5])
    % 隐藏坐标轴数字
    set(gca,'xticklabel',[]);
    set(gca,'yticklabel',[]);
    
%     axis([1.5 10.5 1.5 10.5])%只显示一定范围内的图
    
    set(gca, 'XDir','normal', 'YDir', 'normal'); %设置坐标轴的方向，从左往右数值逐渐增大
    pause(0.0001);% 显示计算过程
end
%% 搜索最小代价点
function index_min_open_list_F = SearchOptimalPoint(open_list, close_list, current_point, end_point)
    %找到最小代价index
    index_min_open_list_F = find(open_list.F == min(open_list.F));
    %如果有多个最小代价值，按一定规则优先选取最优解
    if length(index_min_open_list_F) > 1
        
        if height(close_list) == 1%起点时出现，则优先选取同起始/结束连线夹角最接近者
            index_min_dyaw_end = FindMinDyaw(current_point, end_point, current_point, open_list(index_min_open_list_F, :), 1);
            index_min_open_list_F = index_min_open_list_F(index_min_dyaw_end);
        else % 否则找到与上一刻方向最接近的点
            index_last = find(close_list.key == current_point.parent_key, 1);
            last_point = close_list(index_last, :);
            index_min_dyaw_last = FindMinDyaw(last_point, current_point, current_point, open_list(index_min_open_list_F, :));
            index_min_open_list_F = index_min_open_list_F(index_min_dyaw_last);
            % 如果还有多个结果，优先选取同当前/结束连线夹角最接近者
            if length(index_min_open_list_F) > 1
                index_min_dyaw_end = FindMinDyaw(current_point, end_point, current_point, open_list(index_min_open_list_F, :), 1);
                index_min_open_list_F = index_min_open_list_F(index_min_dyaw_end);
            end
        end
    end
end
%% 从候选点中选取与目标方向最接近的点
function index_min_dyaw = FindMinDyaw(base_point_start, base_point_end, candidate_point_start, candidate_point_end, b_output_single)
    if nargin < 5% 输入值少于五个时
        b_output_single = false;
    end
    end_yaw = atan2(base_point_end.y - base_point_start.y, base_point_end.x - base_point_start.x);
    open_list_yaw = atan2(candidate_point_end.y - candidate_point_start.y, candidate_point_end.x - candidate_point_start.x);
    dyaw = abs(LimitInPi(open_list_yaw - end_yaw));
    if ~b_output_single
        index_min_dyaw = find(dyaw == min(dyaw));
    else
        index_min_dyaw = find(dyaw == min(dyaw), 1, 'last');
    end
end
%%  将输入角度范围限制在+-pi以内
function angle = LimitInPi(angle)
    % 输入：弧度
    % 输出：限制在+-pi以内的弧度
    angle = mod(angle, 2 * pi); % 对2pi取余
    kk = find(abs(angle) > pi);
    if ~isempty(kk)
        angle(kk) = angle(kk) - sign(angle(kk)) * 2 * pi;
    end
end