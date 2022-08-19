clc;clear;
P0 =[0,0];
Pg =[100,100];
% Pb = [10 13;30 26;30 61;40 46;55 59;60 21;81 79;90 90;];
% Pb=[35,82;15,43;59,89;26,39;4,77;75,40;24,81;44,76;69,38;36,22;74,79;39,95;68,33;70,67;44,44;2,83;33,77;42,17;27,86;20,99];%障碍物
Pb = [98,98.5];

P = [Pb;Pg];

Eta_att = 10000;           % 计算引力的增益系数
Eta_rep_ob = 30000;       % 计算斥力的增益系数

d0 = 100;               % 障碍影响距离
n = size(P,1);         % 障碍与目标总计个数
len_step = 0.1;        % 步长
L=20000;                %迭代次数


Pi = P0;
i = 0;
for i=1:L
    
    Path(:,i) = Pi;
    
    for j = 1:n-1
        delta(j,:) = Pi(1,:) - P(j,:);
        dist(j,1) = norm(delta(j,:)); 
        unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % 斥力的单位方向向量
    end
    
    %计算车辆当前位置与目标的单位方向向量、速度向量
    delta(n,:) = P(n,:)-Pi(1,:);                                    %用目标点-车辆点表达引力   
    dist(n,1) = norm(delta(n,:)); 
    unitVector(n,:)=[delta(n,1)/dist(n,1),delta(n,2)/dist(n,1)];

    for j = 1 : n-1
         if dist(j) >= d0
            F_rep_ob(j,:) = [0,0];
         else
            F_rep_ob1_abs = Eta_rep_ob * (1/dist(j,1)-1/d0) * dist(n,1) / dist(j,1)^2;         
            F_rep_ob1 = [F_rep_ob1_abs*unitVector(j,1), F_rep_ob1_abs*unitVector(j,2)];   
            
             
            
            % 改进后的障碍物合斥力计算
            F_rep_ob(j,:) = F_rep_ob1;                                   
         end
    end
  %% 计算合力和方向
    F_rep = [sum(F_rep_ob(:,1))  ,...
           sum(F_rep_ob(:,2)) ];                                      % 所有障碍物的合斥力矢量
    F_att = [Eta_att*dist(n,1)*unitVector(n,1), Eta_att*dist(n,1)*unitVector(n,2)];    % 引力矢量
    F_sum = [F_rep(1,1)+F_att(1,1),F_rep(1,2)+F_att(1,2)];                             % 总合力矢量
    UnitVec_Fsum(i,:) = 1/norm(F_sum) * F_sum;                                         % 总合力的单位向量
    
    %计算车的下一步位置
    Pi(1,1:2)=Pi(1,1:2)+len_step*UnitVec_Fsum(i,:);        
%         if i>2
%         delta_min_last = Pg - Path(:,i-1);%位置终点之间的距离
%         delta_min_current =Pg - Path(:,i);
%         dist_min_last = norm(delta_min_last);
%         dist_min_current = norm(delta_min_current);
%    
%     if (dist_min_current-dist_min_last)>=0%判断陷入局部最小值
%         sita=0-asin(UnitVec_Fsum(i,1));
%         new_sita=sita + pi/6;
%         new_UnitVec_Fsum(1,1)=sin(new_sita);
%         new_UnitVec_Fsum(1,2)=cos(new_sita);
%         Pi(1,1:2)=Pi(1,1:2)+len_step*new_UnitVec_Fsum(1,:);
%         Path(:,i) = Pi;
%         end
%     end


    %判断是否到达终点
    if sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) < 0.1 
        break
    end
end
 Path(:,i)=Pg;            %把路径向量的最后一个点赋值为目标

X=Path(1,:);
Y=Path(2,:);
% gscatter(Pb(:,1),Pb(:,2));
hold on 
plot(Y,X,'.r');%等高线与障碍物点之间关于x，y对称
    
