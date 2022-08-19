clear,clc, close all
global start_point end_point current_point ValueSet 
row = 50;
clm = 50; 
%%
%����ʽ�������ۺ�������
% 1Ϊŷʽ���룻2Ϊ�����پ��룻3Ϊ�Խ��߾��룻4ΪDijkstra�㷨��5����Ȩ�غ�����ŷʽ����
HType = 1;  

W = 0.2; %Ȩ��ϵ��
%% ��ʼ������
% ������洢��һ���ṹ����
NewPoint = @(a, b) struct('x', a, 'y', b); 
% ��x.y����ת��Ϊһ����Ϊ6���ַ���
SetKeyStr = @(x, y) string(num2str([x, y], '%03d'));
% ��ʼ�����
InvalidKeyStr = "000000";
% ����һ���ṹ�壬���ڴ洢ÿһ����״̬
NewSearchPoint = @(a, b) struct('key', SetKeyStr(a, b), 'x', a, 'y', b, 'G', 0, 'H', 0, 'F', 0, 'parent_key', InvalidKeyStr);
% ����һ���ṹ�壬�洢��ͼ��ͬ״̬������Ⱦɫ��
ValueSet = struct('passable', 255, 'impassable', 0, 'openlist', 180, 'closelist', 120, 'start', 60, 'target', 30, 'current', 60, 'path', 60);
% map�е�ֵ�趨
map = ones(row, clm) * ValueSet.passable;
%���õ�ͼ�߽�
[map(1, :), map(end, :), map(:, 1), map(:, end)] = deal(ValueSet.impassable);
%�����ϰ���
load ev2;
ev=1-ev;
map = ev;
map(map==1)=255;% ���ϰ���Ⱦɫ
%% ������ʼ/����������

[start_point, end_point] = deal(NewPoint(2, 2), NewPoint(49, 49)); 
%%  ���ó�ʼ״̬

% Ϊ�����յ���ɫ
[map(start_point.x, start_point.y), map(end_point.x, end_point.y)] = deal(ValueSet.start, ValueSet.target);
% �����㣨��Χ8�����ֱ�Ϊdx dy �Լ��ƶ����ۣ�ʵ��Ϊsqrt(dx*dx+dy*dy)��
SearchDxy = [-1, -1, sqrt(2); 0, -1, 1; 1, -1, sqrt(2); -1, 0, 1; 1, 0, 1; -1, 1, sqrt(2); 0, 1, 1; 1, 1, sqrt(2)];
% ����ʼ�����open_list
open_list(1) = NewSearchPoint(start_point.x, start_point.y); % ��ʼ״̬�趨
%����Ԥ������
open_list(1).H = CalcH(start_point.x, start_point.y, end_point.x, end_point.y, HType,W);
%���ĵ�ǰ����Ϊ0
open_list(1).F = open_list.H + 0 ;
% ��open_list�ӽṹ��ת��Ϊtable����
open_list = struct2table(open_list); %��ȷ�����۵ĵ�
close_list = []; %��ȷ�����۵ĵ�
current_point = open_list; %��ǰ�㣨�趨Ϊ��ʼ�㣩
figure % �򿪻�ͼ

b_find_path = 0; % �Ƿ���·����0Ϊδ����
tic
%% ��ʼѭ����ֱ��open_listΪ�գ��˳�ѭ��

while ~isempty(open_list)% isempty���open_listΪ�����鷵���߼�1
    % ����open_list���ҵ���С�ϴ���F��
    index_min_open_list_F = SearchOptimalPoint(open_list, close_list, current_point, end_point);
    current_point = open_list(index_min_open_list_F, :); %��С����F�ĵ�ѡ��Ϊ��ǰ�㣬���к���open_listѡȡ
    open_list(index_min_open_list_F, :) = []; %��open_list�н���ɾ��
    close_list = [close_list; current_point]; %�������close_list
    map(current_point.x, current_point.y) = ValueSet.closelist; %���¼����close_list���ǣ�Ⱦɫ��
    DrawMap(map); %��ͼ
    % ����Ƿ��ҵ�Ŀ��㣬����ҵ��˳�ѭ��
    if current_point.x == end_point.x && current_point.y == end_point.y
        b_find_path = true;
        break;
    end
    % ��鵱ǰ����Χ�˸����ƶ��㣬�����������ĵ����open_list��
    for search_dxy = SearchDxy'
        search_point = NewSearchPoint(current_point.x + search_dxy(1), current_point.y + search_dxy(2));
        key = SetKeyStr(search_point.x, search_point.y);
        %  ������ǲ��ɵִ�Ļ������� close list �У�������
        if search_point.x <= 0 || search_point.y <= 0 || map(search_point.x, search_point.y) == ValueSet.impassable || map(search_point.x, search_point.y) == ValueSet.closelist
            continue;
        end
        search_point = struct2table(search_point);
        search_point.G = current_point.G + search_dxy(3); %�ƶ�����
        search_point.H = CalcH(search_point.x, search_point.y, end_point.x, end_point.y, HType,W); %Ԥ������g
        search_point.F = search_point.G + search_point.H;% ����õ����ܴ���
        search_point.parent_key = current_point.key;
        index_existed_in_openlist = find(open_list.key == key, 1); %�ж���ǰopen_list���Ƿ���ڸ�������
        % ��������� open list �У��������� open list �����Ұѵ�ǰ��������Ϊ���ĸ��ڵ㣬��¼�÷���� F �� G �� H ֵ
        if map(search_point.x, search_point.y) ~= ValueSet.openlist
            open_list = [open_list; search_point];
            map(search_point.x, search_point.y) = ValueSet.openlist; %���¼����open_list���ǣ�Ⱦɫ��
        else % ������Ѿ��� open list �У��������·�� ( �����ɵ�ǰ���񵽴������� ) �Ƿ���ã��� G ֵ���ο�����С�� G ֵ��ʾ���Ǹ��õ�·��������������������ĸ��ڵ�����Ϊ��ǰ���񣬲����¼������� G �� F ֵ��
            if search_point.G < open_list.G(index_existed_in_openlist)%��open_list�д���ֵ��Gֵ���󣬱�ʾ�ɵ�ǰ�㵽���ֵ���ţ���ԭ����������Ϣ�滻Ϊ��ǰ��������Ϣ
                open_list(index_existed_in_openlist, :) = search_point; % �����滻
            end
        end
    end
end
%% �������·��������parent��Ϣ����·�����������

if b_find_path%�ҵ�·��
    path = close_list(end, :);
    path_step_count = 1;
    map(path.x(1), path.y(1)) = ValueSet.path;
    % һֱ������InvalidKeyStr�����
    while path.parent_key(path_step_count) ~= InvalidKeyStr
        path_step_count = path_step_count + 1;
        index_parent = find(close_list.key == path.parent_key(path_step_count - 1), 1);
        path(path_step_count, :) = close_list(index_parent, :);
        map(path.x(path_step_count), path.y(path_step_count)) = ValueSet.path;
    end
    DrawMap(map); %��ͼ
else
    disp('δ�ҵ���Ч·����');
end
toc
disp(['openlist����: ', num2str(height(open_list)), ', closelist����: ', num2str(height(close_list)), ', �ܴ���: ', num2str(path.G(1), '%3.2f')])
hold on 
x = path.x;
y = path.y;
plot(y,x,'b');

%% Ԥ�����ۼ���
function H = CalcH(x1, y1, x2, y2, type,W)

    dx = x2 - x1;
    dy = y2 - y1;
    switch type
        case 1
            H = sqrt(dx * dx + dy * dy); %ŷʽ���� 
        case 2
            H = abs(dx) + abs(dy); %�����پ���
        case 3
            h_diagonal = min(abs(dx), abs(dy));
            h_straight = abs(dx) + abs(dy);
            H = sqrt(2) * h_diagonal + (h_straight - 2 * h_diagonal); %�Խ��߾��루�б�ѩ����룩
        case 4
            H = 0; %Dijkstra�㷨
        case 5
            H = (dx * dx + dy * dy)*W;%�Ľ���ŷ�Ͼ�����������
    end
end
%% ����mapͼ
function DrawMap(map)
    global start_point end_point current_point ValueSet
    % ע�������map�Ĳ���ֻ��Ϊ����ʾЧ��������Ӱ�쵽�������ڵ�map��
    [map(start_point.x, start_point.y), map(end_point.x, end_point.y), map(current_point.x, current_point.y)] = deal(ValueSet.start, ValueSet.target, ValueSet.current);
     imagesc(map)% imagesc��ʾȾɫ���ĵ�ͼ
     % ����ͼƬɫ��
     colormap hot
     % ��ʾ����
    grid on 
    set(gca,'xtick',[0.5:1:50.5])
    set(gca,'ytick',[0.5:1:50.5])
    % ��������������
    set(gca,'xticklabel',[]);
    set(gca,'yticklabel',[]);
    
%     axis([1.5 10.5 1.5 10.5])%ֻ��ʾһ����Χ�ڵ�ͼ
    
    set(gca, 'XDir','normal', 'YDir', 'normal'); %����������ķ��򣬴���������ֵ������
    pause(0.0001);% ��ʾ�������
end
%% ������С���۵�
function index_min_open_list_F = SearchOptimalPoint(open_list, close_list, current_point, end_point)
    %�ҵ���С����index
    index_min_open_list_F = find(open_list.F == min(open_list.F));
    %����ж����С����ֵ����һ����������ѡȡ���Ž�
    if length(index_min_open_list_F) > 1
        
        if height(close_list) == 1%���ʱ���֣�������ѡȡͬ��ʼ/�������߼н���ӽ���
            index_min_dyaw_end = FindMinDyaw(current_point, end_point, current_point, open_list(index_min_open_list_F, :), 1);
            index_min_open_list_F = index_min_open_list_F(index_min_dyaw_end);
        else % �����ҵ�����һ�̷�����ӽ��ĵ�
            index_last = find(close_list.key == current_point.parent_key, 1);
            last_point = close_list(index_last, :);
            index_min_dyaw_last = FindMinDyaw(last_point, current_point, current_point, open_list(index_min_open_list_F, :));
            index_min_open_list_F = index_min_open_list_F(index_min_dyaw_last);
            % ������ж�����������ѡȡͬ��ǰ/�������߼н���ӽ���
            if length(index_min_open_list_F) > 1
                index_min_dyaw_end = FindMinDyaw(current_point, end_point, current_point, open_list(index_min_open_list_F, :), 1);
                index_min_open_list_F = index_min_open_list_F(index_min_dyaw_end);
            end
        end
    end
end
%% �Ӻ�ѡ����ѡȡ��Ŀ�귽����ӽ��ĵ�
function index_min_dyaw = FindMinDyaw(base_point_start, base_point_end, candidate_point_start, candidate_point_end, b_output_single)
    if nargin < 5% ����ֵ�������ʱ
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
%%  ������Ƕȷ�Χ������+-pi����
function angle = LimitInPi(angle)
    % ���룺����
    % �����������+-pi���ڵĻ���
    angle = mod(angle, 2 * pi); % ��2piȡ��
    kk = find(abs(angle) > pi);
    if ~isempty(kk)
        angle(kk) = angle(kk) - sign(angle(kk)) * 2 * pi;
    end
end