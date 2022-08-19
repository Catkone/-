clc
clear
close all
%% ���ݶ���
k = 4;                                    % k�ס�k-1��B����
flag = 2;                                  %1,2�ֱ���ƾ���B�������ߡ�׼����B��������
load path_A
P =path';

n = size(P,2)-1;                          % n�ǿ��Ƶ��������0��ʼ����

%% ����B��������
path=[];
Bik = zeros(n+1, 1);

if flag == 1     % ����B����
    NodeVector = linspace(0, 1, n+k+1); %�ڵ�m = n+k+1
    for u = 0 : 0.005 : 1-0.005
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path = [path; [p_u(1,1),p_u(2,1)]];
    end
    
elseif flag == 2  % ׼����B����
    NodeVector = U_quasi_uniform(n, k-1); % ׼����B�����Ľڵ�ʸ��
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
% ������������ʾ��Χ
% set(gca, 'YLim',[-4 4]); 

% ����·��
scatter(path(:,1),path(:,2),100, '.b');%·����
% scatter(P(1,:),P(2,:),'g')
plot(path(:,1),path(:,2),'b');
plot(P(1,:),P(2,:),'r');%·����
set(gca, 'xticklabel',[]);
set(gca, 'yticklabel',[]);