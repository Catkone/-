%%  ��ʼ��
clear 
clc
%%  �����ϰ���ͳ�ʼ����Ŀ���

% �����ϰ����ʽ��[min_x, max_x, min_y, max_y]
load obstacle.mat

% �ڵ�ͼ�ϻ����ϰ���
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
% �������
start_x = 2;
start_y = 2;
%��ʼ�����
start_yaw = pi/2;
%�յ�����
goal_x = 48;
goal_y = 48;
%�ڵ�ͼ�ϻ�����ʼ����յ�
plot(start_x, start_y, 'or', 'MarkerSize', 10, 'MarkerFaceColor', 'r');hold on
plot(goal_x, goal_y, 'or', 'MarkerSize', 10, 'MarkerFaceColor', 'r');hold on
%ȥ��������
% set(gca,'xtick',[])
% set(gca,'ytick',[])
%%  ���� open_list �� closed list

open = []; % �洢״̬��Ϣ
open_g = []; % ��ǰ·������g(n)
open_h = []; % Ŀ��Ԥ������h(n)
open_f = []; % g(n)+h(n)
close = []; % ���ڴ洢���ҵ�������·��
%% Ȩ�غ���
w_gn = 1;             % Ԥ��·�����۵�Ȩ��0.2
%%  ����ת��ǶȺͲ�������

steering = [-0.31,-0.15,0.0001,0.15,0.31];%ת��Ƕȣ������ƣ�
arc_length = 1;%��������Ӧ����Ϊ5m
%%  �ڵ�ͼ����������㵽���յ�

% ��ʼ��open_list
% ȫ��id���ٷ��ʵ��ܽڵ㣬�䵱�������Ĺ���
global id
id = 1;
mother_id = 0;

% ����ʼ�����open_list
%open���зֱ�Ϊ[x,y,yaw,g����,���ڵ�,��ǰ�ڵ�id��id,�����ƶ��ľ���]
open = [start_x, start_y, start_yaw, 0, mother_id, id, 0];
% ��������ڵ��
node = [start_x, start_y];
% ���㵱ǰԤ������
open_g = [open_g, pdist([open(1:2);[goal_x, goal_y]])]; % pdist��������ŷ�����(eucliden)��������
% �ܴ���
open_f = open_g + open(end);
counter = 0;
% ��ʱ����ʼ��ʱ
x_global = [];
y_global = [];
tic
while ~isempty(open_f)% ��open_list ����δ�����ļ���ѭ��
    [min_cost,source_ind] = min(open_f); % ��open_list���ҳ���С����ֵ
    source = open(source_ind,:); %ʹ��source�ݴ��ҵ�����Сֵ��
    close = [close; [source, min_cost]];   % ����open_list�ҵ��ĵ����close_list
    if pdist([source(1:2);[goal_x, goal_y]]) < 0.5 % ����Ŀ�����಻��0.5���˳�ѭ��
        break
    end
    open(source_ind,:) = []; % ��open_list��ɾ������close_list�Ľڵ�
    open_g(source_ind) = []; 
    open_f(source_ind) = []; 
    %��չ��һ�ڵ㣬��������չ��·��
    sample = ackermann_sampler(1, source, steering, arc_length,obstacles, node, id);
    if ~isempty(sample)
        %�����ӽڵ����յ��ŷ����þ���
        f = sample(:,1:2) - [goal_x, goal_y];
        f = (f(:,1).^2 + f(:,2).^2).^0.5;
        %����չ���ӽڵ����open����
        open = [open; sample];
        node = [node; sample(:,1:2)];
        %Ԥ��·������
        open_g = [open_g, f.'];
        %·���ܵ����۴���
        open_f = open_g + w_gn*open(:,4).';
        drawnow
    end
end
toc; % ֹͣ��ʱ��
% disp(['Total node explored is ',num2str(length(source(:,1)))]);

%%  ��closed_list�е������·��

[~,min_id] =  max(close(:, 6));
search_id = close(min_id, 5);
path_point = []; % �洢closed_list�е�·��
distance = 0;
while search_id ~= 0       %����С��
    point_id = find(close(:,6)== search_id);%�ҵ����ڵ��Ӧ�Ľڵ�
    path_point = [path_point;[close(point_id,1), close(point_id,2), close(point_id,3)]];
    distance = distance + close(point_id, 7);
    %ʹ��x,y,yaw��Ϣ�������ο������
    draw_car(close(point_id,1), close(point_id,2), close(point_id,3));
    %���£�����ѭ��
    search_id = close(point_id,5);
end
disp(['Total distance travelled is ', num2str(length(path_point(:,1))*arc_length - arc_length)]);
%% ������ʻ·��
for i = length(path_point):-1:2
    %���ڵ�֮��ĺ���Ǳ仯��ֵ
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
   %������ʻ·�ߣ��ֶΣ�
    plot(x_glob, y_glob,'-b','LineWidth',2);hold on
    %����·�ߴ洢����
    x_global = [x_global,x_glob];
    y_global = [y_global,y_glob];
    
end
% plot(x_global,y_global)%����·��ͼ����·�ߣ�

set(gca,'xticklabel',[]);
set(gca,'yticklabel',[]);
hold off
