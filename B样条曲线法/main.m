clc
clear
close all
%% 数据定义
k = 4;                                    % k阶、k-1次B样条
flag = 2;                                  %1,2分别绘制均匀B样条曲线、准均匀B样条曲线
load path_A
P =path';

n = size(P,2)-1;                          % n是控制点个数，从0开始计数

%% 生成B样条曲线
path=[];
Bik = zeros(n+1, 1);

if flag == 1     % 均匀B样条
    NodeVector = linspace(0, 1, n+k+1); %节点m = n+k+1
    for u = 0 : 0.005 : 1-0.005
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path = [path; [p_u(1,1),p_u(2,1)]];
    end
    
elseif flag == 2  % 准均匀B样条
    NodeVector = U_quasi_uniform(n, k-1); % 准均匀B样条的节点矢量
    for u = 0 : 0.005 : 1-0.005
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path=[path; [p_u(1),p_u(2)]];
    end
else
    fprintf('error!\n');
end
figure
hold on
% 设置坐标轴显示范围
% set(gca, 'YLim',[-4 4]); 

% 绘制路径
scatter(path(:,1),path(:,2),100, '.b');%路径点
% scatter(P(1,:),P(2,:),'g')
plot(path(:,1),path(:,2),'b');
plot(P(1,:),P(2,:),'r');%路径点
set(gca, 'xticklabel',[]);
set(gca, 'yticklabel',[]);