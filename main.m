%这个代码里的注释太少了
clc;
clear all;
% 声明全局变量
global Highway Obstacle
global UavTeam
global gcount gfigure
global rm rs ra rd  InitialPosition

% 初始化全局变量
gcount = 0;
gfigure = 1;
time_end = 300;% 模拟结束时间

% 定义管道的参数
rm = 9; % 管道半径
l  = 5; % 无人机长度
vmax = 15; % 最大速度
ro = 0; % 参数 ro，可能为角度或其他控制参数
rs = 20; % 安全半径
ra = 1.5 * rs; % 避障半径
rd = ra + rs + 2 * vmax; % 探测半径

% 初始化 Highway（管道）
Highway(1).ph1 = [0  0]'; 
Highway(1).ph2 = [5000  0]';
Highway(1).rh  = 150; % 管道高度
Highway(1).rb  = 50; % 管道宽度

% 初始化 UavTeam（无人机团队）
M = 40; % 无人机数量
UavTeam = UAVInitialization(M, rm, l, vmax); % 初始化无人机团队
A = [zeros(2 * M, 2 * M) eye(2 * M); zeros(2 * M, 2 * M) l * eye(2 * M)];
B = [zeros(2 * M, 2 * M); -l * eye(2 * M)];
C = eye(4 * M);
D = zeros(4 * M, 2 * M);
Initialcondition = [InitialPosition; zeros(2 * M, 1)];

% 设置所有无人机的感知半径和避障半径
for k=1:M
UavTeam.Uav(k).rs = rs;
UavTeam.Uav(k).ra = ra;
end

% 设置第一个无人机的初始航线，靠近管道边缘
UavTeam.Uav(1).Waypoint = [0;149.9];
UavTeam.Uav(1).HomePos = UavTeam.Uav(1).Waypoint;
UavTeam.Uav(1).CurrentPos= UavTeam.Uav(1).Waypoint;
% UavTeam.Uav(1).vmax = 20;
% % 
% % % % Two multicopters are close to each other
% UavTeam.Uav(2).Waypoint = [-500;0.1];
% UavTeam.Uav(2).HomePos = UavTeam.Uav(2).Waypoint;
% UavTeam.Uav(2).CurrentPos= UavTeam.Uav(2).Waypoint;
% % UavTeam.Uav(k).vmax = 50;
% 
% UavTeam.Uav(3).Waypoint = [-500;-0.1];
% UavTeam.Uav(3).HomePos = UavTeam.Uav(3).Waypoint;
% UavTeam.Uav(3).CurrentPos= UavTeam.Uav(3).Waypoint;


% Draw 2D map
% 绘制初始地图
figure(1);
% subplot(3,2,1)
MyMap(UavTeam,Obstacle,Highway);
 
%  for k = 1: M
%     o = [UavTeam.Uav(k).CurrentPos(1) UavTeam.Uav(k).CurrentPos(2)]';
%     mydrawcolorball(o(1),o(2),k);   
% end
 tic
'start'
sim('platform.slx')
'over'
toc
% 绘制结果
figure(2)

% 绘制第一个子图：无人机与管道边缘的距离随时间的变化
subplot(2, 1, 2)
plot(mindis(1:end - 2000, 1), mindis(1:end - 2000, 3), '-', 'LineWidth', 2); hold on;
ksm = find(mindis(:, 3) == min(mindis(:, 3))); % 找到最小距离值对应的索引
text(mindis(ksm(1), 1), mindis(ksm(1), 3), ['\leftarrow' num2str(mindis(ksm(1), 3))], 'HorizontalAlignment', 'left') % 在图上标注最小距离值
plot(mindis(1:end - 2000, 1), rs * ones(size(mindis(1:end - 2000, 1))), 'r-.', 'LineWidth', 1); % 绘制管道边缘距离的参考线
legend('Between two multicopters') % 添加图例
axis([0 230 0 60]) % 设置坐标轴范围
xlabel('t(sec)') % 设置 x 轴标签
ylabel('Between multicopter and tunnel edge(m)') % 设置 y 轴标签

% 绘制第二个子图：无人机之间的最小距离随时间的变化
subplot(2, 1, 1)
plot(mindis(1:end - 2000, 1), mindis(1:end - 2000, 4), '-', 'LineWidth', 2); hold on;
ksh = find(mindis(:, 4) == min(mindis(:, 4))); % 找到最小距离值对应的索引
text(mindis(ksh(1), 1), mindis(ksh(1), 4), ['\leftarrow' num2str(mindis(ksh(1), 4))], 'HorizontalAlignment', 'left') % 在图上标注最小距离值
plot(mindis(1:end - 2000, 1), 2 * rs * ones(size(mindis(1:end - 2000, 1))), 'r-.', 'LineWidth', 1); % 绘制最小距离的参考线
xlabel('t(sec)') % 设置 x 轴标签
ylabel('minimum distance(m)') % 设置 y 轴标签
axis([0 230 0 60]) % 设置坐标轴范围
legend('Minimum distance(m)') % 添加图例

hold off


    