clc;clear;
global len_step
P0 =[0,0];
Pg =[100,100];
% Pb = [1 1.3;3 2.6;3 6.1;4 4.6;5.5 5.9;6 2.1;8.1 7.9;9 9;];%路径
% Pb=[3.5,8.2;1.5,4.3;5.9,8.9;2.6,3.9;0.4,7.7;7.5,4;2.4,8.1;4.4,7.6;6.9,3.8;3.6,2.2;7.4,7.9;3.9,9.5;6.8,3.3;7,6.7;4.4,4.4;0.2,8.3;3.3,7.7;4.2,1.7;2.7,8.6;2,9.9];
Pb = [99,99];
% Pb=round((rand(30,2)*100 ))/10;%生成随机路径
% Pb = [5,2.5;
%     5,3;
%     5,3.5;
%     5,4;
%     5,4.5;
%     5,5;
%     5,5.5;
%     5,6;
%     ];

P = [Pb;Pg];
Eta_att = 20;           % 计算引力的增益系数
Eta_rep_ob = 50;       % 计算斥力的增益系数

d0 = 50;               % 障碍影响距离
n = size(P,1);         % 障碍与目标总计个数
len_step = 0.1;        % 步长
L=50000;                %迭代次数
m=2;                    %斥力修正因子

Pi = P0;
i = 0;
for i=1:L 
    
    
    for j = 1:n-1
        delta(j,:) = Pi(1,:) - P(j,:);
        dist(j,1) = norm(delta(j,:)); %norm返回欧几里得长度
        unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % 斥力的单位方向向量
    end
    
    %计算车辆当前位置与目标的单位方向向量、速度向量
    delta(n,:) = P(n,:)-Pi(1,:);                                    %用目标点-车辆点表达引力   
    dist(n,1) = norm(delta(n,:)); 
    unitVector(n,:)=[delta(n,1)/dist(n,1),delta(n,2)/dist(n,1)];%单位向量

    for j = 1 : n-1
         if dist(j) >= d0
            F_rep_ob(j,:) = [0,0];  %斥力大于斥力影响范围为0
         else
            F_rep_ob1_abs = Eta_rep_ob * (1/dist(j,1)-1/d0) * dist(n,1)^(m) / delta(j,1)^2;         
            F_rep_ob1 = [F_rep_ob1_abs*unitVector(j,1), F_rep_ob1_abs*unitVector(j,2)];%计算斥力   
            
            F_rep_ob2_abs = 0.5 * Eta_rep_ob * (1/dist(j,1) - 1/d0)^2 * m *delta(n,1)^(m-1);                
            F_rep_ob2 = [F_rep_ob2_abs * unitVector(j,1), F_rep_ob2_abs * unitVector(j,2)]; %计算斥力势场 
            
            % 改进后的障碍物合斥力计算
%             F_rep_ob(j,:) = F_rep_ob1+F_rep_ob2;
%             F_rep_ob(j,:) = F_rep_ob1+F_rep_ob2;   
            F_rep_ob(j,:) = F_rep_ob1;      

         end
    end
  %% 计算合力和方向
    F_rep(i,:) = [sum(F_rep_ob(:,1)) ,sum(F_rep_ob(:,2)) ]; % 所有障碍物的合斥力矢量
    F_att(i,:) = [Eta_att*dist(n,1)*unitVector(n,1), Eta_att*dist(n,1)*unitVector(n,2)]; % 引力矢量
    unit_F_rep(i,:) = 1/ norm(F_rep(i,1)) * F_rep(i,2);
    unit_F_rep(isnan(unit_F_rep)) = 0;%将nan数据置为0
    F_sum(i,:) = [F_rep(i,1)+F_att(i,1),F_rep(i,2)+F_att(i,2)]; % 总合力矢量
    F_sum_he(i,:) = sqrt(F_sum(i,1)^2 + F_sum(i,2)^2);
    UnitVec_Fsum(i,:) = 1/norm(F_sum(i,:)) * F_sum(i,:);  % 总合力的单位向量
%     
%     if i>2 %改步长
%         if F_rep(i,:)~=0 %& (abs(F_sum_he(i-1)-F_sum_he(i)) >100)
%             len_step = 0.01;
% %         elseif F_rep(i,:)~=0 & (abs(F_sum_he(i-1)-F_sum_he(i)) >20)
% %             len_step = 0.001;
% %         else
%             len_step = 0.1;
%         end
%     end
    
% if i>2
%     if F_rep(i,:)~=0 & (abs(F_sum_he(i-1)-F_sum_he(i)) >100)
%         
%     end
% end
    
    
    %计算车的下一步位置
    Pi(1,1:2)=Pi(1,1:2)+len_step*UnitVec_Fsum(i,:);   
    
    Path(:,i) = Pi;
    %判断局部最小值
%     if ((dot(v1,v2)/norm(v1)/norm(v2))==1)&&( %障碍物与小车前进方向相同，且合力为0 认为陷入局部最小值
%         
%     end
    if i>2
        delta_min_last = Pg - Path(:,i-1);%位置终点之间的距离
        delta_min_current =Pg - Path(:,i);
        dist_min_last = norm(delta_min_last);
        dist_min_current = norm(delta_min_current);
   
    if (dist_min_current-dist_min_last)>=0%判断陷入局部最小值
        sita=0-asin(UnitVec_Fsum(i,1));
        new_sita=sita - pi/6
        new_UnitVec_Fsum(1,1)=sin(new_sita);
        new_UnitVec_Fsum(1,2)=cos(new_sita);
        Pi(1,1:2)=Pi(1,1:2)+len_step*new_UnitVec_Fsum(1,:);
        Path(:,i) = Pi;
        end
    end
    %判断是否到达终点
    if sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) < 0.1 
        break
    end
end
Path(:,i)=Pg;            %把路径向量的最后一个点赋值为目标

X=Path(1,:);
Y=Path(2,:);

plot(X,Y,'.r');