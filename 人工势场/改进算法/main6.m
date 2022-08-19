clc;clear;
global len_step
P0 =[0,0];
Pg =[100,100];
% Pb = [1 1.3;3 2.6;3 6.1;4 4.6;5.5 5.9;6 2.1;8.1 7.9;9 9;];%·��
% Pb=[3.5,8.2;1.5,4.3;5.9,8.9;2.6,3.9;0.4,7.7;7.5,4;2.4,8.1;4.4,7.6;6.9,3.8;3.6,2.2;7.4,7.9;3.9,9.5;6.8,3.3;7,6.7;4.4,4.4;0.2,8.3;3.3,7.7;4.2,1.7;2.7,8.6;2,9.9];
Pb = [99,99];
% Pb=round((rand(30,2)*100 ))/10;%�������·��
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
Eta_att = 20;           % ��������������ϵ��
Eta_rep_ob = 50;       % �������������ϵ��

d0 = 50;               % �ϰ�Ӱ�����
n = size(P,1);         % �ϰ���Ŀ���ܼƸ���
len_step = 0.1;        % ����
L=50000;                %��������
m=2;                    %������������

Pi = P0;
i = 0;
for i=1:L 
    
    
    for j = 1:n-1
        delta(j,:) = Pi(1,:) - P(j,:);
        dist(j,1) = norm(delta(j,:)); %norm����ŷ����ó���
        unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % �����ĵ�λ��������
    end
    
    %���㳵����ǰλ����Ŀ��ĵ�λ�����������ٶ�����
    delta(n,:) = P(n,:)-Pi(1,:);                                    %��Ŀ���-������������   
    dist(n,1) = norm(delta(n,:)); 
    unitVector(n,:)=[delta(n,1)/dist(n,1),delta(n,2)/dist(n,1)];%��λ����

    for j = 1 : n-1
         if dist(j) >= d0
            F_rep_ob(j,:) = [0,0];  %�������ڳ���Ӱ�췶ΧΪ0
         else
            F_rep_ob1_abs = Eta_rep_ob * (1/dist(j,1)-1/d0) * dist(n,1)^(m) / delta(j,1)^2;         
            F_rep_ob1 = [F_rep_ob1_abs*unitVector(j,1), F_rep_ob1_abs*unitVector(j,2)];%�������   
            
            F_rep_ob2_abs = 0.5 * Eta_rep_ob * (1/dist(j,1) - 1/d0)^2 * m *delta(n,1)^(m-1);                
            F_rep_ob2 = [F_rep_ob2_abs * unitVector(j,1), F_rep_ob2_abs * unitVector(j,2)]; %��������Ƴ� 
            
            % �Ľ�����ϰ���ϳ�������
%             F_rep_ob(j,:) = F_rep_ob1+F_rep_ob2;
%             F_rep_ob(j,:) = F_rep_ob1+F_rep_ob2;   
            F_rep_ob(j,:) = F_rep_ob1;      

         end
    end
  %% ��������ͷ���
    F_rep(i,:) = [sum(F_rep_ob(:,1)) ,sum(F_rep_ob(:,2)) ]; % �����ϰ���ĺϳ���ʸ��
    F_att(i,:) = [Eta_att*dist(n,1)*unitVector(n,1), Eta_att*dist(n,1)*unitVector(n,2)]; % ����ʸ��
    unit_F_rep(i,:) = 1/ norm(F_rep(i,1)) * F_rep(i,2);
    unit_F_rep(isnan(unit_F_rep)) = 0;%��nan������Ϊ0
    F_sum(i,:) = [F_rep(i,1)+F_att(i,1),F_rep(i,2)+F_att(i,2)]; % �ܺ���ʸ��
    F_sum_he(i,:) = sqrt(F_sum(i,1)^2 + F_sum(i,2)^2);
    UnitVec_Fsum(i,:) = 1/norm(F_sum(i,:)) * F_sum(i,:);  % �ܺ����ĵ�λ����
%     
%     if i>2 %�Ĳ���
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
    
    
    %���㳵����һ��λ��
    Pi(1,1:2)=Pi(1,1:2)+len_step*UnitVec_Fsum(i,:);   
    
    Path(:,i) = Pi;
    %�жϾֲ���Сֵ
%     if ((dot(v1,v2)/norm(v1)/norm(v2))==1)&&( %�ϰ�����С��ǰ��������ͬ���Һ���Ϊ0 ��Ϊ����ֲ���Сֵ
%         
%     end
    if i>2
        delta_min_last = Pg - Path(:,i-1);%λ���յ�֮��ľ���
        delta_min_current =Pg - Path(:,i);
        dist_min_last = norm(delta_min_last);
        dist_min_current = norm(delta_min_current);
   
    if (dist_min_current-dist_min_last)>=0%�ж�����ֲ���Сֵ
        sita=0-asin(UnitVec_Fsum(i,1));
        new_sita=sita - pi/6
        new_UnitVec_Fsum(1,1)=sin(new_sita);
        new_UnitVec_Fsum(1,2)=cos(new_sita);
        Pi(1,1:2)=Pi(1,1:2)+len_step*new_UnitVec_Fsum(1,:);
        Path(:,i) = Pi;
        end
    end
    %�ж��Ƿ񵽴��յ�
    if sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) < 0.1 
        break
    end
end
Path(:,i)=Pg;            %��·�����������һ���㸳ֵΪĿ��

X=Path(1,:);
Y=Path(2,:);

plot(X,Y,'.r');