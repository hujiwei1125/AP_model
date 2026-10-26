function [trajPosNEU_m_f, trajVelNEU_mps_f, trajAccNEU_mps2_f, trajJerkNEU_mps3_f] = trajGeneratorNEU(curPosNEU_m_f, curVelNEU_mps_f, curAccNEU_mps2_f, preWpNEU_m_f, curWpNEU_m_f, nextWpNEU_m_f, ...
    velSpNEU_mps_f, posVelValid_vec, isHoverWp_b, forceStop_b, segSwitch_b, newOriOldNEU_m_f, dt_s_f, reset_b, paramNE_f, paramU_f, mcSegMssn_u8)
%#codegen
%
% 功能：
% 生成NEU三轴jerk受限平滑轨迹，并进行N/E/U三轴S曲线时间同步。
% 原始速度目标按照位置有效性结构分为3D位置、2D水平位置、1D垂向
% 位置和纯速度四类；方向向量统一通过unit_or_zero生成。
%
% velSpNEU_mps_f：
% 1. 对应位置无效时，为纯速度目标；
% 2. 对应位置有效时，为位置导引速度上的速度前馈。
%
% posVelValid_vec：[posNEValid; velNEValid; posUValid; velUValid]
%
% paramNE_f：
% 1 最大水平jerk，[m/s^3]
% 2 最大水平加速度，[m/s^2]
% 3 最大水平速度，[m/s]
% 4 水平巡航速度，[m/s]
% 5 最大允许水平轨迹误差，[m]
% 6 水平航点接受半径，[m]
% 7 最小前视距离，[m]
% 8 转弯判断最小轨迹速度，[m/s]
% 9 转弯判断方向余弦阈值
% 10 前倾转换阶段旋翼水平速度，[m/s]
%
% paramU_f：
% 1 最大垂向jerk，[m/s^3]
% 2 爬升轨迹使用的最大垂向加速度幅值，[m/s^2]
% 3 下降轨迹使用的最大垂向加速度幅值，[m/s^2]
% 4 最大爬升速度，[m/s]
% 5 最大下降速度幅值，[m/s]
% 6 最大允许垂向轨迹误差，[m]
% 7 垂向航点接受半径，[m]
%
% 垂向非对称约束：
% 1. 根据未平滑垂向速度目标rawVelSpNEU_mps_f(3)的方向，选择一整套
%    垂向速度和加速度幅值约束；
% 2. 选定后，单轴S曲线规划器内部仍使用对称的+/-maxAcceleration约束；
% 3. rawVelSpU为0时保持上一套约束，避免停车附近反复切换。

%% 常量
% smallValue_f用于轨迹规划中的除零、退化方向和近零判断。

smallValue_f = single(1.0e-6);
floatEpsilon_f = single(1.1920929e-7);
clampAccuracy_mps_f = single(0.5);

%% 最小持久状态
% 轨迹生成器每个周期都会根据当前状态重新规划S曲线，并在同一周期推进dt。
% 因此只保存推进后的物理状态，不再跨周期保存jerk、规划初值、方向、
% T1/T2/T3和localTime。
persistent trajAccStateNEU_mps2_f trajVelStateNEU_mps_f trajPosStateNEU_m_f
persistent selectedUConstraintModeState_i8
persistent previousMaxSpeedNEState_mps_f
persistent initialized_b

if isempty(initialized_b)
    initialized_b = false;

    trajAccStateNEU_mps2_f = single(zeros(3,1));
    trajVelStateNEU_mps_f = single(zeros(3,1));
    trajPosStateNEU_m_f = single(zeros(3,1));
    selectedUConstraintModeState_i8 = int8(0);
    previousMaxSpeedNEState_mps_f = single(0.0);
else
    initialized_b = true;
end

%% 参数读取
maxJerkNE_mps3_f = max(paramNE_f(1), smallValue_f);
maxAccNE_mps2_f = max(paramNE_f(2), smallValue_f);
maxVelNE_mps_f = max(paramNE_f(3), smallValue_f);
cruiseSpeedNE_mps_f = constrain_scalar(paramNE_f(4), single(0.0), maxVelNE_mps_f);
maxTrajErrorNE_m_f = max(paramNE_f(5), smallValue_f);
horizonAptRadius_m_f = max(paramNE_f(6), smallValue_f);
minLookaheadDist_m_f = max(paramNE_f(7), horizonAptRadius_m_f);
turnMinSpeed_mps_f = max(paramNE_f(8), single(0.0));
turnCosThreshold_f = constrain_scalar(paramNE_f(9), single(-1.0), single(1.0));
fwdFixedSpd_mps_f = max(paramNE_f(10), smallValue_f);

maxJerkU_mps3_f = max(paramU_f(1), smallValue_f);
maxAccUp_mps2_f = max(paramU_f(2), smallValue_f);
maxAccDown_mps2_f = max(paramU_f(3), smallValue_f);
maxVelUp_mps_f = max(paramU_f(4), smallValue_f);
maxVelDown_mps_f = max(paramU_f(5), smallValue_f);
maxTrajErrorU_m_f = max(paramU_f(6), smallValue_f);
verticalAcceptanceRadius_m_f = max(paramU_f(7), smallValue_f);

axisMaxJerkNEU_mps3_f = single([maxJerkNE_mps3_f; maxJerkNE_mps3_f; maxJerkU_mps3_f]);

dtEff_s_f = max(dt_s_f, single(0.0));

%% 输入有效性
posNEValid_b = logical(posVelValid_vec(1));
velNEValid_b = logical(posVelValid_vec(2));
posUValid_b = logical(posVelValid_vec(3));
velUValid_b = logical(posVelValid_vec(4));

%% 初始化、复位或航段坐标系切换
if reset_b || (initialized_b == false)
    trajAccStateNEU_mps2_f = curAccNEU_mps2_f;
    trajVelStateNEU_mps_f = curVelNEU_mps_f;
    trajPosStateNEU_m_f = curPosNEU_m_f;

    trajAccStateNEU_mps2_f(1) = constrain_scalar(curAccNEU_mps2_f(1), -maxAccNE_mps2_f, maxAccNE_mps2_f);
    trajAccStateNEU_mps2_f(2) = constrain_scalar(curAccNEU_mps2_f(2), -maxAccNE_mps2_f, maxAccNE_mps2_f);
    trajAccStateNEU_mps2_f(3) = constrain_scalar(curAccNEU_mps2_f(3), -maxAccDown_mps2_f, maxAccUp_mps2_f);

    trajVelStateNEU_mps_f(1) = constrain_scalar(curVelNEU_mps_f(1), -maxVelNE_mps_f, maxVelNE_mps_f);
    trajVelStateNEU_mps_f(2) = constrain_scalar(curVelNEU_mps_f(2), -maxVelNE_mps_f, maxVelNE_mps_f);
    trajVelStateNEU_mps_f(3) = constrain_scalar(curVelNEU_mps_f(3), -maxVelDown_mps_f, maxVelUp_mps_f);

    % 1表示爬升约束，-1表示下降约束，0表示尚未选择方向并采用保守值。
    if curVelNEU_mps_f(3) > smallValue_f
        selectedUConstraintModeState_i8 = int8(1);
    elseif curVelNEU_mps_f(3) < -smallValue_f
        selectedUConstraintModeState_i8 = int8(-1);
    else
        selectedUConstraintModeState_i8 = int8(0);
    end

    previousMaxSpeedNEState_mps_f = single(0.0);

elseif segSwitch_b
    % 各航段坐标轴方向不变，仅平移原点
    trajPosStateNEU_m_f = trajPosStateNEU_m_f - newOriOldNEU_m_f;
else
    % 不处理
end

%% 按有效性组合生成原始速度目标

% 先装载速度输入。位置有效时，它是位置速度的前馈；位置无效时，
% 它就是直接送给S曲线平滑器的纯速度目标。无效速度分量保持为0。
rawVelSpNEU_mps_f = single(zeros(3,1));

if velNEValid_b
    rawVelSpNEU_mps_f(1:2) = velSpNEU_mps_f(1:2);
end

if velUValid_b
    rawVelSpNEU_mps_f(3) = velSpNEU_mps_f(3);
end

if forceStop_b
    % 强制停车优先级最高，忽略位置目标和速度前馈。
    rawVelSpNEU_mps_f(:) = single(0.0);

elseif posNEValid_b && posUValid_b
    %% NE和U位置同时有效：生成三维位置速度目标
    waypointsNEU_m_f = [preWpNEU_m_f, curWpNEU_m_f, nextWpNEU_m_f];

    [crossingPointNEU_m_f, targetSpeedNE_mps_f, previousMaxSpeedNEState_mps_f] = ...
        get_xy_guidance_constraints_handle(trajPosStateNEU_m_f, trajVelStateNEU_mps_f, waypointsNEU_m_f, ...
        isHoverWp_b, mcSegMssn_u8, cruiseSpeedNE_mps_f, fwdFixedSpd_mps_f, maxJerkNE_mps3_f, ...
        maxAccNE_mps2_f, horizonAptRadius_m_f, minLookaheadDist_m_f, turnMinSpeed_mps_f, ...
        turnCosThreshold_f, previousMaxSpeedNEState_mps_f, smallValue_f);

    if isHoverWp_b
        nextWpForUSpeed_m_f = curWpNEU_m_f(3);
    else
        nextWpForUSpeed_m_f = nextWpNEU_m_f(3);
    end

    maxAllowedSpeedU_mps_f = get_max_z_speed_handle(trajPosStateNEU_m_f(3), curWpNEU_m_f(3), ...
        nextWpForUSpeed_m_f, maxJerkU_mps3_f, maxAccUp_mps2_f, maxAccDown_mps2_f, ...
        maxVelUp_mps_f, maxVelDown_mps_f, verticalAcceptanceRadius_m_f, smallValue_f);

    posToCrossingNEU_m_f = crossingPointNEU_m_f - trajPosStateNEU_m_f;
    directionToCrossingNEU_f = unit_or_zero(posToCrossingNEU_m_f, smallValue_f);
    combinedSpeed_mps_f = sqrt(targetSpeedNE_mps_f * targetSpeedNE_mps_f + maxAllowedSpeedU_mps_f * maxAllowedSpeedU_mps_f);
    posVelSpNEU_mps_f = directionToCrossingNEU_f * combinedSpeed_mps_f;

    % 先限制水平模长，再限制垂向模长。
    % 0.5是小速度区间判定精度accuracy，不是另一方向的固定缩放系数。
    posVelSpNEU_mps_f = clamp_to_xy_norm(posVelSpNEU_mps_f, targetSpeedNE_mps_f, clampAccuracy_mps_f, floatEpsilon_f);
    posVelSpNEU_mps_f = clamp_to_u_norm(posVelSpNEU_mps_f, maxAllowedSpeedU_mps_f, clampAccuracy_mps_f, floatEpsilon_f);

    if velNEValid_b
        rawVelSpNEU_mps_f(1:2) = rawVelSpNEU_mps_f(1:2) + posVelSpNEU_mps_f(1:2);
    else
        rawVelSpNEU_mps_f(1:2) = posVelSpNEU_mps_f(1:2);
    end

    if velUValid_b
        rawVelSpNEU_mps_f(3) = rawVelSpNEU_mps_f(3) + posVelSpNEU_mps_f(3);
    else
        rawVelSpNEU_mps_f(3) = posVelSpNEU_mps_f(3);
    end

elseif posNEValid_b
    %% 仅NE位置有效：生成二维水平位置速度目标
    waypointsNEU_m_f = [preWpNEU_m_f, curWpNEU_m_f, nextWpNEU_m_f];

    [crossingPointNEU_m_f, targetSpeedNE_mps_f, previousMaxSpeedNEState_mps_f] = ...
        get_xy_guidance_constraints_handle(trajPosStateNEU_m_f, trajVelStateNEU_mps_f, waypointsNEU_m_f, ...
        isHoverWp_b, mcSegMssn_u8, cruiseSpeedNE_mps_f, fwdFixedSpd_mps_f, maxJerkNE_mps3_f, ...
        maxAccNE_mps2_f, horizonAptRadius_m_f, minLookaheadDist_m_f, turnMinSpeed_mps_f, ...
        turnCosThreshold_f, previousMaxSpeedNEState_mps_f, smallValue_f);

    posToCrossingNE_m_f = crossingPointNEU_m_f(1:2) - trajPosStateNEU_m_f(1:2);
    directionToCrossingNE_f = unit_or_zero(posToCrossingNE_m_f, smallValue_f);
    posVelSpNE_mps_f = directionToCrossingNE_f * targetSpeedNE_mps_f;

    if velNEValid_b
        rawVelSpNEU_mps_f(1:2) = rawVelSpNEU_mps_f(1:2) + posVelSpNE_mps_f;
    else
        rawVelSpNEU_mps_f(1:2) = posVelSpNE_mps_f;
    end

elseif posUValid_b
    %% 仅U位置有效：生成一维垂向位置速度目标
    if isHoverWp_b
        nextWpForUSpeed_m_f = curWpNEU_m_f(3);
    else
        nextWpForUSpeed_m_f = nextWpNEU_m_f(3);
    end

    maxAllowedSpeedU_mps_f = get_max_z_speed_handle(trajPosStateNEU_m_f(3), curWpNEU_m_f(3), ...
        nextWpForUSpeed_m_f, maxJerkU_mps3_f, maxAccUp_mps2_f, maxAccDown_mps2_f, ...
        maxVelUp_mps_f, maxVelDown_mps_f, verticalAcceptanceRadius_m_f, smallValue_f);

    posErrorU_m_f = curWpNEU_m_f(3) - trajPosStateNEU_m_f(3);
    posVelSpU_mps_f = single(0.0);

    if abs(posErrorU_m_f) > smallValue_f
        posVelSpU_mps_f = sign(posErrorU_m_f) * maxAllowedSpeedU_mps_f;
    end

    if velUValid_b
        rawVelSpNEU_mps_f(3) = rawVelSpNEU_mps_f(3) + posVelSpU_mps_f;
    else
        rawVelSpNEU_mps_f(3) = posVelSpU_mps_f;
    end

else
    % 不处理，直接使用前馈速度
end

% 位置速度和速度前馈相加后，统一执行最终性能包线保护。
rawVelSpNEU_mps_f(1:2) = limit_vector_2d(rawVelSpNEU_mps_f(1:2), maxVelNE_mps_f, smallValue_f);     % 超长放缩
rawVelSpNEU_mps_f(3) = constrain_scalar(rawVelSpNEU_mps_f(3), -maxVelDown_mps_f, maxVelUp_mps_f);

%% 选择当前垂向整套约束
% NEU中U向上为正：rawVelSpU>0选择爬升约束，rawVelSpU<0选择下降约束。
if rawVelSpNEU_mps_f(3) > smallValue_f
    selectedUConstraintModeState_i8 = int8(1);
elseif rawVelSpNEU_mps_f(3) < -smallValue_f
    selectedUConstraintModeState_i8 = int8(-1);
else
    % 保持上一周期所选模式，使停车S曲线沿用进入停车前的整套参数。
end

if selectedUConstraintModeState_i8 > int8(0)
    selectedMaxAccU_mps2_f = maxAccUp_mps2_f;
    selectedMaxVelU_mps_f = maxVelUp_mps_f;
elseif selectedUConstraintModeState_i8 < int8(0)
    selectedMaxAccU_mps2_f = maxAccDown_mps2_f;
    selectedMaxVelU_mps_f = maxVelDown_mps_f;
else
    selectedMaxAccU_mps2_f = min(maxAccUp_mps2_f, maxAccDown_mps2_f);
    selectedMaxVelU_mps_f = min(maxVelUp_mps_f, maxVelDown_mps_f);
end

axisMaxAccPlanNEU_mps2_f = single([maxAccNE_mps2_f; maxAccNE_mps2_f; selectedMaxAccU_mps2_f]);
axisMaxVelPlanNEU_mps_f = single([maxVelNE_mps_f; maxVelNE_mps_f; selectedMaxVelU_mps_f]);

%% 计算三轴公共时间拉伸系数
% 水平：只有虚拟轨迹处于实际飞机运动前方时才进行时间拉伸。
timeStretchNE_f = single(1.0);
droneToTrajNE_m_f = trajPosStateNEU_m_f(1:2) - curPosNEU_m_f(1:2);
trajSpeedNE_mps_f = norm(trajVelStateNEU_mps_f(1:2));
trajectoryAheadNE_b = trajSpeedNE_mps_f > smallValue_f && dot(droneToTrajNE_m_f, trajVelStateNEU_mps_f(1:2)) > single(0.0);

if trajectoryAheadNE_b
    normalizedErrorNE_f = norm(droneToTrajNE_m_f) / maxTrajErrorNE_m_f;
    timeStretchNE_f = single(1.0) - constrain_scalar(normalizedErrorNE_f, single(0.0), single(1.0));
end

% 垂向：误差方向与轨迹速度方向一致，说明虚拟轨迹处于实际飞机前方。
timeStretchU_f = single(1.0);
droneToTrajU_m_f = trajPosStateNEU_m_f(3) - curPosNEU_m_f(3);
trajectoryAheadU_b = abs(trajVelStateNEU_mps_f(3)) > smallValue_f && droneToTrajU_m_f * trajVelStateNEU_mps_f(3) > single(0.0);

if trajectoryAheadU_b
    normalizedErrorU_f = abs(droneToTrajU_m_f) / maxTrajErrorU_m_f;
    timeStretchU_f = single(1.0) - constrain_scalar(normalizedErrorU_f, single(0.0), single(1.0));
end

% 三轴必须使用同一个有效时间推进，否则会破坏同步后的T1/T2/T3关系。
timeStretchNEU_f = min(timeStretchNE_f, timeStretchU_f);

%% 根据当前状态和本周期原始速度目标规划S曲线
trajVelSpNEU_mps_f = single(zeros(3,1));
trajDirectionNEU_f = single(zeros(3,1));
trajT1NEU_s_f = single(zeros(3,1));
trajT2NEU_s_f = single(zeros(3,1));
trajT3NEU_s_f = single(zeros(3,1));

for axisIdx_i = 1:3
    [trajVelSpNEU_mps_f(axisIdx_i), trajDirectionNEU_f(axisIdx_i), ...
        trajT1NEU_s_f(axisIdx_i), trajT2NEU_s_f(axisIdx_i), trajT3NEU_s_f(axisIdx_i)] = ...
        update_axis_durations(trajAccStateNEU_mps2_f(axisIdx_i), trajVelStateNEU_mps_f(axisIdx_i), rawVelSpNEU_mps_f(axisIdx_i), axisMaxVelPlanNEU_mps_f(axisIdx_i), ...
        axisMaxAccPlanNEU_mps2_f(axisIdx_i), axisMaxJerkNEU_mps3_f(axisIdx_i), smallValue_f);
end

%% N/E/U三轴时间同步
[trajT1NEU_s_f, trajT2NEU_s_f, trajT3NEU_s_f] = synchronize_trajectory_time_neu(trajT1NEU_s_f, trajT2NEU_s_f, trajT3NEU_s_f, trajAccStateNEU_mps2_f, trajVelStateNEU_mps_f, ...
trajVelSpNEU_mps_f, trajDirectionNEU_f, axisMaxAccPlanNEU_mps2_f, axisMaxJerkNEU_mps3_f, smallValue_f);

%% 用本周期刚得到的同步S曲线推进dt，并保存最小物理状态
trajJerkNEU_mps3_f = single(zeros(3,1));

for axisIdx_i = 1:3
    [trajJerkNEU_mps3_f(axisIdx_i), trajAccStateNEU_mps2_f(axisIdx_i), ...
        trajVelStateNEU_mps_f(axisIdx_i), trajPosStateNEU_m_f(axisIdx_i)] = ...
        update_trajectory_axis(trajAccStateNEU_mps2_f(axisIdx_i), trajVelStateNEU_mps_f(axisIdx_i), ...
        trajPosStateNEU_m_f(axisIdx_i), trajT1NEU_s_f(axisIdx_i), trajT2NEU_s_f(axisIdx_i), ...
        trajT3NEU_s_f(axisIdx_i), axisMaxJerkNEU_mps3_f(axisIdx_i), trajDirectionNEU_f(axisIdx_i), ...
        dtEff_s_f, timeStretchNEU_f);
end

%% 输出本周期推进后的轨迹状态
trajPosNEU_m_f = trajPosStateNEU_m_f;
trajVelNEU_mps_f = trajVelStateNEU_mps_f;
trajAccNEU_mps2_f = trajAccStateNEU_mps2_f;

end

function maxAllowedSpeedU_mps_f = get_max_z_speed_handle(trajPosU_m_f, curWpU_m_f, nextWpU_m_f, maxJerkU_mps3_f, maxAccUp_mps2_f, maxAccDown_mps2_f, ...
    maxVelUp_mps_f, maxVelDown_mps_f, verticalAcceptanceRadius_m_f, smallValue_f)
%#codegen
%
% 根据位置生成速度目标的方向，选择同方向轨迹所使用的整套加速度约束。
% 选中后，get_max_z_speed和后续S曲线均把该值作为对称加速度幅值。
% 不能因为爬升末段需要负加速度刹车，就切换为下降轨迹约束。

posErrorU_m_f = curWpU_m_f - trajPosU_m_f;

if posErrorU_m_f > smallValue_f
    maxAccSelectedU_mps2_f = maxAccUp_mps2_f;
elseif posErrorU_m_f < -smallValue_f
    maxAccSelectedU_mps2_f = maxAccDown_mps2_f;
else
    % 当前高度误差很小时，根据下一航段速度目标方向选择约束。
    nextSegmentDeltaU_m_f = nextWpU_m_f - curWpU_m_f;

    if nextSegmentDeltaU_m_f > smallValue_f
        maxAccSelectedU_mps2_f = maxAccUp_mps2_f;
    elseif nextSegmentDeltaU_m_f < -smallValue_f
        maxAccSelectedU_mps2_f = maxAccDown_mps2_f;
    else
        maxAccSelectedU_mps2_f = min(maxAccUp_mps2_f, maxAccDown_mps2_f);
    end
end

maxAllowedSpeedU_mps_f = get_max_z_speed(trajPosU_m_f, curWpU_m_f, nextWpU_m_f, maxJerkU_mps3_f, maxAccSelectedU_mps2_f, maxVelUp_mps_f, ...
    maxVelDown_mps_f, verticalAcceptanceRadius_m_f, smallValue_f);

end

function [crossingPointNEU_m_f, targetSpeedNE_mps_f, previousMaxSpeedNEOut_mps_f] = ...
    get_xy_guidance_constraints_handle(trajPosNEU_m_f, trajVelNEU_mps_f, waypointsNEU_m_f, ...
    isHoverWp_b, mcSegMssn_u8, cruiseSpeedNE_mps_f, fwdFixedSpd_mps_f, maxJerkNE_mps3_f, ...
    maxAccNE_mps2_f, horizonAptRadius_m_f, minLookaheadDist_m_f, turnMinSpeed_mps_f, ...
    turnCosThreshold_f, previousMaxSpeedNEIn_mps_f, smallValue_f)
%#codegen
%
% 统一生成3D和2D位置分支共用的水平追踪点及允许速度。
% isHoverWp_b和前倾转换均直接追踪当前航点，其余航段允许使用crossing point。

curWpNEU_m_f = waypointsNEU_m_f(:,2);
isSingleWaypoint_b = isHoverWp_b || mcSegMssn_u8 == uint8(5);

if isHoverWp_b
    crossingPointNEU_m_f = curWpNEU_m_f;
    distanceToTargetNE_m_f = norm(curWpNEU_m_f(1:2) - trajPosNEU_m_f(1:2));
    maxStopSpeedNE_mps_f = compute_max_speed_from_distance(maxJerkNE_mps3_f, maxAccNE_mps2_f, ...
        distanceToTargetNE_m_f, single(0.0), smallValue_f);
    targetSpeedNE_mps_f = min(cruiseSpeedNE_mps_f, maxStopSpeedNE_mps_f);

elseif mcSegMssn_u8 == uint8(5)
    % 前倾转换阶段保持直接追踪转换目标，并使用专用水平速度。
    crossingPointNEU_m_f = curWpNEU_m_f;
    targetSpeedNE_mps_f = fwdFixedSpd_mps_f;

else
    crossingPointNEU_m_f = get_crossing_point(trajPosNEU_m_f, trajVelNEU_mps_f, ...
        waypointsNEU_m_f, horizonAptRadius_m_f, minLookaheadDist_m_f, turnMinSpeed_mps_f, turnCosThreshold_f, smallValue_f);
    targetSpeedNE_mps_f = cruiseSpeedNE_mps_f;
end

turning_b = is_turning(trajPosNEU_m_f, trajVelNEU_mps_f, curWpNEU_m_f, ...
    horizonAptRadius_m_f, turnMinSpeed_mps_f, turnCosThreshold_f, smallValue_f);

if ~isSingleWaypoint_b && turning_b
    % 在转弯过程中不允许本周期目标速度高于转弯前锁存值。
    % 若在运动中复位导致锁存值尚未建立，则先用当前候选值初始化。
    if previousMaxSpeedNEIn_mps_f > smallValue_f
        targetSpeedNE_mps_f = min(previousMaxSpeedNEIn_mps_f, targetSpeedNE_mps_f);
    end

    previousMaxSpeedNEOut_mps_f = targetSpeedNE_mps_f;
else
    previousMaxSpeedNEOut_mps_f = targetSpeedNE_mps_f;
end

end

function outputNEU_mps_f = clamp_to_xy_norm(inputNEU_mps_f, maxNormNE_mps_f, accuracy_mps_f, floatEpsilon_f)
%#codegen
%
% 1. 一般超限时按同一比例缩放整个NEU向量，保持三维方向不变；
% 2. 当水平上限和当前水平模长均小于accuracy时，只缩放NE分量，
%    避免微小水平数值误差把有效U速度一并缩小；

outputNEU_mps_f = inputNEU_mps_f;
normNEU_mps_f = norm(inputNEU_mps_f(1:2));
scaleFactor_f = single(1.0);

if normNEU_mps_f > floatEpsilon_f
    scaleFactor_f = maxNormNE_mps_f / normNEU_mps_f;
end

if scaleFactor_f < single(1.0)
    if maxNormNE_mps_f < accuracy_mps_f && normNEU_mps_f < accuracy_mps_f
        outputNEU_mps_f(1:2) = inputNEU_mps_f(1:2) * scaleFactor_f;
    else
        outputNEU_mps_f = inputNEU_mps_f * scaleFactor_f;
    end
end

end

function outputNEU_mps_f = clamp_to_u_norm(inputNEU_mps_f, maxNormU_mps_f, accuracy_mps_f, floatEpsilon_f)
%#codegen
%
% 1. 一般超限时按同一比例缩放整个NEU向量，保持三维方向不变；
% 2. 当垂向上限和当前垂向模长均小于accuracy时，只缩放U分量，
%    避免微小垂向数值误差把有效NE速度一并缩小。

outputNEU_mps_f = inputNEU_mps_f;
absSpeedU_mps_f = abs(inputNEU_mps_f(3));
scaleFactor_f = single(1.0);

if absSpeedU_mps_f > floatEpsilon_f
    scaleFactor_f = maxNormU_mps_f / absSpeedU_mps_f;
end

if scaleFactor_f < single(1.0)
    if maxNormU_mps_f < accuracy_mps_f && absSpeedU_mps_f < accuracy_mps_f
        outputNEU_mps_f(3) = inputNEU_mps_f(3) * scaleFactor_f;
    else
        outputNEU_mps_f = inputNEU_mps_f * scaleFactor_f;
    end
end

end
