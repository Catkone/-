clc;clear;
P0 =[0,0];
Pg =[100,100];
Pb = [10 13;30 26;30 61;40 46;55 59;60 21;81 79;90 90;];
P = [Pb;Pg];

Eta_att = 10;           % ��������������ϵ��
Eta_rep_ob = 3000;       % �������������ϵ��

d0 = 1000;               % �ϰ�Ӱ�����
n = size(P,1);         % �ϰ���Ŀ���ܼƸ���
len_step = 0.1;        % ����
L=10000;                %��������


Pi = P0;
i = 0;
for i=1:L
    
    Path(:,i) = Pi;
    
    for j = 1:n-1
        delta(j,:) = Pi(1,:) - P(j,:);
        dist(j,1) = norm(delta(j,:)); 
        unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % �����ĵ�λ��������
    end
    
    %���㳵����ǰλ����Ŀ��ĵ�λ�����������ٶ�����
    delta(n,:) = P(n,:)-Pi(1,:);                                    %��Ŀ���-������������   
    dist(n,1) = norm(delta(n,:)); 
    unitVector(n,:)=[delta(n,1)/dist(n,1),delta(n,2)/dist(n,1)];

    for j = 1 : n-1
         if dist(j) >= d0
            F_rep_ob(j,:) = [0,0];
         else
            F_rep_ob1_abs = Eta_rep_ob * (1/dist(j,1)-1/d0) * dist(n,1) / dist(j,1)^2;         
            F_rep_ob1 = [F_rep_ob1_abs*unitVector(j,1), F_rep_ob1_abs*unitVector(j,2)];   
            
             
            
            % �Ľ�����ϰ���ϳ�������
            F_rep_ob(j,:) = F_rep_ob1;                                   
         end
    end
  %% ��������ͷ���
    F_rep = [sum(F_rep_ob(:,1))  ,...
           sum(F_rep_ob(:,2)) ];                                      % �����ϰ���ĺϳ���ʸ��
    F_att = [Eta_att*dist(n,1)*unitVector(n,1), Eta_att*dist(n,1)*unitVector(n,2)];    % ����ʸ��
    F_sum = [F_rep(1,1)+F_att(1,1),F_rep(1,2)+F_att(1,2)];                             % �ܺ���ʸ��
    UnitVec_Fsum(i,:) = 1/norm(F_sum) * F_sum;                                         % �ܺ����ĵ�λ����
    
    %���㳵����һ��λ��
    Pi(1,1:2)=Pi(1,1:2)+len_step*UnitVec_Fsum(i,:);                     

    %�ж��Ƿ񵽴��յ�
    if sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) < 1 
        break
    end
end
 Path(:,i)=Pg;            %��·�����������һ���㸳ֵΪĿ��

X=Path(1,:);
Y=Path(2,:);
% gscatter(Pb(:,1),Pb(:,2));
hold on 
plot(100,100,'v',0,0,'ms',Y,X,'.r');%�ȸ������ϰ����֮�����x��y�Գ�
    
