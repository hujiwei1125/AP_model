%% demo_trajectoryGeneratorNEU_route_switch
% 统一NEU三轴轨迹生成器多航段测试。
%
% 测试目的：
% 1. 使用单个trajectoryGeneratorNEU同时生成N/E/U轨迹；
% 2. 水平和垂向使用不同jerk、acc、vel约束；
% 3. 每周期对N/E/U三轴S曲线进行总时间同步；
% 4. 三轴使用统一timeStretch推进；
% 5. 验证以当前航段目标点为局部原点时的航段切换坐标变换；
% 6. 最终航点减速到零并悬停。
%
% 本demo使用理想跟踪：飞机实际状态直接取轨迹生成器输出。

clear;
clc;
close all;
clear trajectoryGeneratorNEU;

%% 1. 控制周期与约束参数

dt_s_f = single(0.05);
maxSimTime_s_f = single(800.0);
maxStep_u32 = uint32(ceil(double(maxSimTime_s_f / dt_s_f)));

% 水平参数：
% [最大jerk, 最大加速度, 最大速度, 巡航速度, 最大轨迹误差,
%  水平接受半径, 最小前视距离, 转弯最小速度, 转弯方向余弦阈值]
paramNE_f = single([3.0; 2.0; 10.0; 8.0; 5.0; 5.0; 6.0; 0.5; 0.98]);

% 垂向参数：
% [最大jerk, 最大加速度, 最大爬升速度, 最大下降速度,
%  最大轨迹误差, 垂向接受半径]
paramU_f = single([2.5; 1.5; 3.0; 2.0; 3.0; 3.0]);

horizontalAcceptanceRadius_m_f = paramNE_f(6);
verticalAcceptanceRadius_m_f = paramU_f(6);

%% 2. 全局NEU航路

waypointsGlobalNEU_m_f = single([0.0, 0.0, 200.0, 300.0, 500.0; ...
                                 0.0, 0.0, -10.0, -50.0, 0.0; ...
                                 0.0, 20.0, 30.0, 40.0, 0.0]);
waypointCount_u32 = uint32(size(waypointsGlobalNEU_m_f, 2));
segmentCount_u32 = waypointCount_u32 - uint32(1);

%% 3. 飞机状态与局部坐标系初始化

curPosGlobalNEU_m_f = waypointsGlobalNEU_m_f(:,1);
curVelGlobalNEU_mps_f = single(zeros(3,1));
curAccGlobalNEU_mps2_f = single(zeros(3,1));

segmentIdx_u32 = uint32(1);
currentOriginGlobalNEU_m_f = waypointsGlobalNEU_m_f(:,2);
segmentSwitch_b = false;
newOriginOldNEU_m_f = single(zeros(3,1));
reset_b = true;

%% 4. 日志预分配

timeLog_s_f = zeros(double(maxStep_u32),1,'single');
segmentLog_u32 = zeros(double(maxStep_u32),1,'uint32');
trajPosGlobalLogNEU_m_f = zeros(double(maxStep_u32),3,'single');
trajVelLogNEU_mps_f = zeros(double(maxStep_u32),3,'single');
trajAccLogNEU_mps2_f = zeros(double(maxStep_u32),3,'single');
trajJerkLogNEU_mps3_f = zeros(double(maxStep_u32),3,'single');
rawVelSpLogNEU_mps_f = zeros(double(maxStep_u32),3,'single');
timeStretchLog_f = zeros(double(maxStep_u32),1,'single');
synchronizedTimeLog_s_f = zeros(double(maxStep_u32),1,'single');
switchTime_s_f = single([]);
switchWaypointIdx_u32 = uint32([]);
lastStep_u32 = uint32(0);

%% 5. 主仿真循环

for step_u32 = uint32(1):maxStep_u32
    time_s_f = single(double(step_u32 - uint32(1)) * double(dt_s_f));

    prevWpIdx_u32 = segmentIdx_u32;
    targetWpIdx_u32 = segmentIdx_u32 + uint32(1);
    nextWpIdx_u32 = min(segmentIdx_u32 + uint32(2), waypointCount_u32);

    prevWpGlobalNEU_m_f = waypointsGlobalNEU_m_f(:,prevWpIdx_u32);
    targetWpGlobalNEU_m_f = waypointsGlobalNEU_m_f(:,targetWpIdx_u32);
    nextWpGlobalNEU_m_f = waypointsGlobalNEU_m_f(:,nextWpIdx_u32);

    prevWpLocalNEU_m_f = prevWpGlobalNEU_m_f - currentOriginGlobalNEU_m_f;
    targetWpLocalNEU_m_f = targetWpGlobalNEU_m_f - currentOriginGlobalNEU_m_f;
    nextWpLocalNEU_m_f = nextWpGlobalNEU_m_f - currentOriginGlobalNEU_m_f;

    curPosLocalNEU_m_f = curPosGlobalNEU_m_f - currentOriginGlobalNEU_m_f;
    curVelLocalNEU_mps_f = curVelGlobalNEU_mps_f;
    curAccLocalNEU_mps2_f = curAccGlobalNEU_mps2_f;

    isFinalSegment_b = segmentIdx_u32 == segmentCount_u32;
    isHoverWaypoint_b = isFinalSegment_b;

    forceStop_b = false;
    if isFinalSegment_b
        nearFinalNE_b = norm(curPosLocalNEU_m_f(1:2)) < single(0.25);
        nearFinalU_b = abs(curPosLocalNEU_m_f(3)) < single(0.25);
        forceStop_b = nearFinalNE_b && nearFinalU_b;
    end

    [trajPosLocalNEU_m_f, trajVelNEU_mps_f, trajAccNEU_mps2_f, trajJerkNEU_mps3_f, rawVelSpNEU_mps_f, timeStretchNEU_f, synchronizedTime_s_f] = trajectoryGeneratorNEU(curPosLocalNEU_m_f, curVelLocalNEU_mps_f, curAccLocalNEU_mps2_f, prevWpLocalNEU_m_f, targetWpLocalNEU_m_f, nextWpLocalNEU_m_f, isHoverWaypoint_b, forceStop_b, segmentSwitch_b, newOriginOldNEU_m_f, dt_s_f, reset_b, paramNE_f, paramU_f);

    reset_b = false;
    segmentSwitch_b = false;
    newOriginOldNEU_m_f(:) = single(0.0);

    trajPosGlobalNEU_m_f = trajPosLocalNEU_m_f + currentOriginGlobalNEU_m_f;

    % 理想跟踪。
    curPosGlobalNEU_m_f = trajPosGlobalNEU_m_f;
    curVelGlobalNEU_mps_f = trajVelNEU_mps_f;
    curAccGlobalNEU_mps2_f = trajAccNEU_mps2_f;

    logIdx_i = double(step_u32);
    timeLog_s_f(logIdx_i) = time_s_f;
    segmentLog_u32(logIdx_i) = segmentIdx_u32;
    trajPosGlobalLogNEU_m_f(logIdx_i,:) = trajPosGlobalNEU_m_f.';
    trajVelLogNEU_mps_f(logIdx_i,:) = trajVelNEU_mps_f.';
    trajAccLogNEU_mps2_f(logIdx_i,:) = trajAccNEU_mps2_f.';
    trajJerkLogNEU_mps3_f(logIdx_i,:) = trajJerkNEU_mps3_f.';
    rawVelSpLogNEU_mps_f(logIdx_i,:) = rawVelSpNEU_mps_f.';
    timeStretchLog_f(logIdx_i) = timeStretchNEU_f;
    synchronizedTimeLog_s_f(logIdx_i) = synchronizedTime_s_f;
    lastStep_u32 = step_u32;

    distanceToTargetNE_m_f = norm(curPosGlobalNEU_m_f(1:2) - targetWpGlobalNEU_m_f(1:2));
    distanceToTargetU_m_f = abs(curPosGlobalNEU_m_f(3) - targetWpGlobalNEU_m_f(3));
    targetReached_b = distanceToTargetNE_m_f <= horizontalAcceptanceRadius_m_f && distanceToTargetU_m_f <= verticalAcceptanceRadius_m_f;

    if targetReached_b && ~isFinalSegment_b
        oldOriginGlobalNEU_m_f = currentOriginGlobalNEU_m_f;
        segmentIdx_u32 = segmentIdx_u32 + uint32(1);
        currentOriginGlobalNEU_m_f = waypointsGlobalNEU_m_f(:,segmentIdx_u32 + uint32(1));
        newOriginOldNEU_m_f = currentOriginGlobalNEU_m_f - oldOriginGlobalNEU_m_f;
        segmentSwitch_b = true;

        switchTime_s_f(end+1,1) = time_s_f;
        switchWaypointIdx_u32(end+1,1) = segmentIdx_u32;
        fprintf('t = %6.2f s: 切换到航段 WP%d -> WP%d\n', double(time_s_f), double(segmentIdx_u32), double(segmentIdx_u32 + uint32(1)));
    end

    if isFinalSegment_b && targetReached_b
        horizontalSpeed_mps_f = norm(curVelGlobalNEU_mps_f(1:2));
        verticalSpeed_mps_f = abs(curVelGlobalNEU_mps_f(3));
        if horizontalSpeed_mps_f < single(0.08) && verticalSpeed_mps_f < single(0.05)
            fprintf('t = %6.2f s: 到达最终悬停航点 WP%d，仿真结束。\n', double(time_s_f), double(waypointCount_u32));
            break;
        end
    end
end

%% 6. 截取有效数据

lastIdx_i = double(lastStep_u32);
timeLog_s_f = timeLog_s_f(1:lastIdx_i);
segmentLog_u32 = segmentLog_u32(1:lastIdx_i);
trajPosGlobalLogNEU_m_f = trajPosGlobalLogNEU_m_f(1:lastIdx_i,:);
trajVelLogNEU_mps_f = trajVelLogNEU_mps_f(1:lastIdx_i,:);
trajAccLogNEU_mps2_f = trajAccLogNEU_mps2_f(1:lastIdx_i,:);
trajJerkLogNEU_mps3_f = trajJerkLogNEU_mps3_f(1:lastIdx_i,:);
rawVelSpLogNEU_mps_f = rawVelSpLogNEU_mps_f(1:lastIdx_i,:);
timeStretchLog_f = timeStretchLog_f(1:lastIdx_i);
synchronizedTimeLog_s_f = synchronizedTimeLog_s_f(1:lastIdx_i);

%% 7. 绘图

figure('Name','NEU三维轨迹');
plot3(double(trajPosGlobalLogNEU_m_f(:,2)), double(trajPosGlobalLogNEU_m_f(:,1)), double(trajPosGlobalLogNEU_m_f(:,3)), 'LineWidth', 1.5);
hold on;
plot3(double(waypointsGlobalNEU_m_f(2,:)), double(waypointsGlobalNEU_m_f(1,:)), double(waypointsGlobalNEU_m_f(3,:)), 'o--', 'LineWidth', 1.0);
grid on;
axis equal;
xlabel('E / m');
ylabel('N / m');
zlabel('U / m');
title('统一NEU三轴轨迹生成器');
legend('生成轨迹','航路点','Location','best');

figure('Name','NE水平轨迹');
plot(double(trajPosGlobalLogNEU_m_f(:,2)), double(trajPosGlobalLogNEU_m_f(:,1)), 'LineWidth', 1.5);
hold on;
plot(double(waypointsGlobalNEU_m_f(2,:)), double(waypointsGlobalNEU_m_f(1,:)), 'o--', 'LineWidth', 1.0);
grid on;
axis equal;
xlabel('E / m');
ylabel('N / m');
title('trajectoryGeneratorNE 多航段水平轨迹');
legend('生成轨迹','航路点','Location','best');

figure('Name','NEU位置');
plot(double(timeLog_s_f), double(trajPosGlobalLogNEU_m_f), 'LineWidth', 1.2);
grid on;
xlabel('时间 / s');
ylabel('位置 / m');
title('NEU轨迹位置');
legend('N','E','U','Location','best');

figure('Name','速度规划');
plot(double(timeLog_s_f), double(trajVelLogNEU_mps_f), 'LineWidth', 1.2);
hold on;
plot(double(timeLog_s_f), double(rawVelSpLogNEU_mps_f), '--');
grid on;
xlabel('时间 / s');
ylabel('速度 / (m/s)');
title('S曲线轨迹速度与原始速度目标');
legend('traj V_N','traj V_E','traj V_U','raw V_N','raw V_E','raw V_U','Location','best');

figure('Name','加速度');
plot(double(timeLog_s_f), double(trajAccLogNEU_mps2_f), 'LineWidth', 1.1);
grid on;
xlabel('时间 / s');
ylabel('加速度 / (m/s^2)');
title('NEU轨迹加速度');
legend('a_N','a_E','a_U','Location','best');

figure('Name','Jerk');
plot(double(timeLog_s_f), double(trajJerkLogNEU_mps3_f), 'LineWidth', 1.1);
grid on;
xlabel('时间 / s');
ylabel('jerk / (m/s^3)');
title('NEU轨迹jerk');
legend('j_N','j_E','j_U','Location','best');

figure('Name','时间同步状态');
plot(double(timeLog_s_f), double(timeStretchLog_f), 'LineWidth', 1.2);
hold on;
plot(double(timeLog_s_f), double(synchronizedTimeLog_s_f), 'LineWidth', 1.2);
grid on;
xlabel('时间 / s');
ylabel('数值');
title('公共时间拉伸与三轴同步总时间');
legend('timeStretch_{NEU}','T_{sync}','Location','best');

%% 8. 结果摘要

fprintf('\n===== Demo结果摘要 =====\n');
fprintf('有效仿真时长: %.2f s\n', double(timeLog_s_f(end)));
fprintf('执行航段数量: %d\n', double(max(segmentLog_u32)));
fprintf('最终位置 N/E/U: %.3f, %.3f, %.3f m\n', double(curPosGlobalNEU_m_f(1)), double(curPosGlobalNEU_m_f(2)), double(curPosGlobalNEU_m_f(3)));
fprintf('最终速度 N/E/U: %.3f, %.3f, %.3f m/s\n', double(curVelGlobalNEU_mps_f(1)), double(curVelGlobalNEU_mps_f(2)), double(curVelGlobalNEU_mps_f(3)));

if ~isempty(switchTime_s_f)
    fprintf('航段切换次数: %d\n', numel(switchTime_s_f));
    for switchIdx_i = 1:numel(switchTime_s_f)
        fprintf('  第%d次切换: t=%.2f s, 新航段从WP%d开始\n', switchIdx_i, double(switchTime_s_f(switchIdx_i)), double(switchWaypointIdx_u32(switchIdx_i)));
    end
end
