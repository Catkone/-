clear;clc;close all;
P0 =[0,0];
Pg =[100,100];
x=[0:1:140];
y=[0:1:140];

% %% �ϰ�ͼ
% load ev2;            
% ev=2-ev;
% field = ev;
% img1 = edge(ev,'canny');%��߽�
% [h,w] = size(ev);
% [row,col]=find(img1==1);
% Pb=[col,row];

%% ���ϰ�
% Pb = [10 13;30 26;30 61;40 46;55 59;60 21;81 79;90 90;];
% Pb=[35,82;15,43;59,89;26,39;4,77;75,40;24,81;44,76;69,38;36,22;74,79;39,95;68,33;70,67;44,44;2,83;33,77;42,17;27,86;20,99];%�ϰ���
Pb = [98,98.5];
%% ��������
P = [Pb;Pg];
Eta_att = 10;           % ��������������ϵ��
Eta_rep_ob = 3000;       % �������������ϵ��

d0 = 100;               % �ϰ�Ӱ�����
n = size(P,1);         % �ϰ���Ŀ���ܼƸ���
len_step = 0.1;        % ����
L=10000;                %��������
m=2;                    %������������
U_rep_ob = [0,0];

for y_i=1:length(x)
    for x_i=1:length(y)

         Pi{x_i,y_i} = [x(x_i),y(y_i)];

    end
end

for y_i=1:length(x)
    for x_i=1:length(y)
        
    for j = 1:n-1
        delta(j,:) = Pi{x_i,y_i} - P(j,:);
        dist(j,:) = norm(delta(j,:)); %norm����ŷ����þ���
        unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % �����ĵ�λ��������
    end
    
    %���㳵����ǰλ����Ŀ��ĵ�λ�����������ٶ�����
    delta(n,:) = P(n,:)-Pi{x_i,y_i};                                    %��Ŀ���-������������   
    dist(n,1) = norm(delta(n,:)); 
    unitVector(n,:)=[delta(n,1)/dist(n,1),delta(n,2)/dist(n,1)];%������λ����

    for j = 1 : n-1
         if dist(j) >= d0
            U_rep_ob(j,:) = [0,0];  %�������ڳ���Ӱ�췶ΧΪ0
         else
            U_rep_ob(j,:)=0.5*Eta_rep_ob*(1/dist(j,:) -1/d0);  
            U_rep_ob2(j,:)=0.5 * Eta_rep_ob * (1/dist(j,1) - 1/d0)^2 * m *delta(n,1)^(m-1);  
    
            U_rep_ob(j,:) =  U_rep_ob(j,:) + U_rep_ob2(j,:);
%             U_rep_ob(j,:) =  U_rep_ob(j,:);
         end     
    end
  %% ��������ͷ���
   F_rep{x_i,y_i} = [sum(U_rep_ob(:,1)) ,sum(U_rep_ob(:,2)) ]; % �����ϰ���ĺϳ���ʸ��

    F_att{x_i,y_i} = [Eta_att*dist(n,1)*unitVector(n,1), Eta_att*dist(n,1)*unitVector(n,2)]; % ����ʸ��
  
    U_sum = F_att{x_i,y_i} + F_rep{x_i,y_i};
    U_rep = F_rep{x_i,y_i};
    U_att = F_att{x_i,y_i};
    
    U_he(x_i,y_i) = sqrt(U_sum(:,1)^2+U_sum(:,2)^2);
    U_re(x_i,y_i) = sqrt(U_rep(:,1)^2+U_rep(:,2)^2);
    U_at(x_i,y_i) = sqrt(U_att(:,1)^2+U_att(:,2)^2);
    
    end
end
U_he(find(isnan(U_he)==1)) = 0;
U_he(find(U_he > 2000)) =2000;

U_re(find(isnan(U_re)==1)) = 0;
U_re(find(U_re > 300)) =300;

U_at(find(isnan(U_at)==1)) = 0;
U_at(find(U_at > 1500)) =1500;
% 
% pcolor(x,y,U_he);
% shading interp;
% colorbar;
% % colormap(hsv);%����ɫ��ͼ
% figure(2)
% contour(x,y,U_he,50);%�ȸ���ͼ
% 
% % Path(:,i)=Pg;            %��·�����������һ���㸳ֵΪĿ��
contour(U_he)
hold on 
% gscatter(Pb(:,2),Pb(:,1));

