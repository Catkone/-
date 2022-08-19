%%  初始化
clear 
clc
%%  创建障碍物和初始点与目标点

% 导入障碍物，格式如[min_x, max_x, min_y, max_y]
load obstacle.mat

% 在地图上画出障碍物
obstacles = [];
        for i = 1:1:length(obstacle(:,1))
            min_x = obstacle(i,1) ;
            max_x = obstacle(i,2) ;
            min_y = obstacle(i,3) ;
            max_y = obstacle(i,4) ;
            obstacles = [obstacles; [min_x, max_x, min_y, max_y]];
            obs_x = [min_x, max_x, max_x, min_x, min_x];
            obs_y = [min_y, min_y, max_y, max_y, min_y];
            fill(obs_x, obs_y,'k'); hold on
            clear min_x;
            clear max_x;
            clear min_y;
            clear max_y;
        end
% 起点坐标
start_x = 2;
start_y = 2;
%初始航向角
start_yaw = pi/2;
%终点坐标
goal_x = 48;
goal_y = 48;
%在地图上画出起始点和终点
plot(start_x, start_y, 'or', 'MarkerSize', 10, 'MarkerFaceColor', 'r');hold on
plot(goal_x, goal_y, 'or', 'MarkerSize', 10, 'MarkerFaceColor', 'r');hold on
%去掉坐标轴
% set(gca,'xtick',[])
% set(gca,'ytick',[])
%%  创建 open_list 和 closed list

open = []; % 存储状态信息
open_g = []; % 当前路径代价g(n)
open_h = []; % 目标预估代价h(n)
open_f = []; % g(n)+h(n)
close = []; % 用于存储所找到的最优路径
%% 权重函数
w_gn = 1;             % 预估路径代价的权重0.2
%%  创建转向角度和采样距离

steering = [-0.31,-0.15,0.0001,0.15,0.31];%转向角度（弧度制）
arc_length = 1;%步长，对应距离为5m
%%  在地图上搜索从起点到达终点

% 初始化open_list
% 全局id跟踪访问的总节点，充当计数器的功能
global id
id = 1;
mother_id = 0;

% 将起始点放入open_list
%open表中分别为[x,y,yaw,g代价,父节点,当前节点id，id,代表移动的距离]
open = [start_x, start_y, start_yaw, 0, mother_id, id, 0];
% 将起点放入节点表
node = [start_x, start_y];
% 计算当前预估代价
open_g = [open_g, pdist([open(1:2);[goal_x, goal_y]])]; % pdist函数返回欧几里得(eucliden)启发函数
% 总代价
open_f = open_g + open(end);
counter = 0;
% 计时器开始计时
x_global = [];
y_global = [];
tic
while ~isempty(open_f)% 当open_list 还有未遍历的继续循环
    [min_cost,source_ind] = min(open_f); % 从open_list中找出最小代价值
    source = open(source_ind,:); %使用source暂存找到的最小值点
    close = [close; [source, min_cost]];   % 将从open_list找到的点放入close_list
    if pdist([source(1:2);[goal_x, goal_y]]) < 0.5 % 若与目标点相距不到0.5则退出循环
        break
    end
    open(source_ind,:) = []; % 从open_list中删除放入close_list的节点
    open_g(source_ind) = []; 
    open_f(source_ind) = []; 
    %拓展下一节点，并画出拓展的路线
    sample = ackermann_sampler(1, source, steering, arc_length,obstacles, node, id);
    if ~isempty(sample)
        %计算子节点与终点的欧几里得距离
        f = sample(:,1:2) - [goal_x, goal_y];
        f = (f(:,1).^2 + f(:,2).^2).^0.5;
        %将拓展的子节点放入open表中
        open = [open; sample];
        node = [node; sample(:,1:2)];
        %预估路径代价
        open_g = [open_g, f.'];
        %路径总的评价代价
        open_f = open_g + w_gn*open(:,4).';
        drawnow
    end
end
toc; % 停止定时器
% disp(['Total node explored is ',num2str(length(source(:,1)))]);

%%  从closed_list中导出最短路径

[~,min_id] =  max(close(:, 6));
search_id = close(min_id, 5);
path_point = []; % 存储closed_list中的路径
distance = 0;
while search_id ~= 0       %画出小车
    point_id = find(close(:,6)== search_id);%找到父节点对应的节点
    path_point = [path_point;[close(point_id,1), close(point_id,2), close(point_id,3)]];
    distance = distance + close(point_id, 7);
    %使用x,y,yaw信息画出矩形块代表车辆
    draw_car(close(point_id,1), close(point_id,2), close(point_id,3));
    %更新，继续循环
    search_id = close(point_id,5);
end
disp(['Total distance travelled is ', num2str(length(path_point(:,1))*arc_length - arc_length)]);
%% 画出行驶路线
for i = length(path_point):-1:2
    %两节点之间的航向角变化差值
    alpha = path_point(i-1,3) - path_point(i,3);
    turning_radius = arc_length/alpha;
    arc = linspace(0.01, arc_length, 8);
    data_x = 0;
    data_y = 0;
    for j = 1:1:length(arc)
        alpha = arc(j)/turning_radius;
        data_x(j) = turning_radius*sin(alpha);
        data_y(j) = turning_radius*(1-cos(alpha));
    end
    x_glob = path_point(i,1) + data_x*cos(path_point(i,3)) - data_y*sin(path_point(i,3));
    y_glob = path_point(i,2) + data_x*sin(path_point(i,3)) + data_y*cos(path_point(i,3));
   %画出行驶路线（分段）
    plot(x_glob, y_glob,'-b','LineWidth',2);hold on
    %将总路线存储下来
    x_global = [x_global,x_glob];
    y_global = [y_global,y_glob];
    
end
% plot(x_global,y_global)%画出路线图（总路线）

set(gca,'xticklabel',[]);
set(gca,'yticklabel',[]);
hold off
