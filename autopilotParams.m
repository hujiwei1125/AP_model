%% 通用参数
apParams.dt_s_f = single(0.05);
apParams.G_mps2_f = single(9.81);

maxNumWp = uint8(30);
wp_struct = struct('wpType_u8',WP_TYPE.INVALID_WP,'lat_deg_d',double(0),'lon_deg_d',double(0),'alt_m_f',single(0),...
    'spd_mps_f',single(0),'radius_m_f',single(0),'fastWp_b',false,'yawType_u8',uint8(0),'yaw_deg_f',single(0) ...
    ,'timeOrCount_u16',uint16(0),'arrivalCond_u8',uint8(0));
defaultRoute_struct = repmat(wp_struct, maxNumWp, 1);
apParams.defaultRoute_struct = defaultRoute_struct;

%% 旋翼水平轨迹生成器参数
apParams.trajGenParams.maxJerkNE_mps3_f = single(2.0);     % 旋翼水平最大加加速度幅值，单位m/s^3
apParams.trajGenParams.cruiseSpdNE_mps_f = single(7.0);     % 旋翼水平巡航速度，单位m/s
apParams.trajGenParams.maxTrajErrNE_m_f = single(2.0);     % 最大允许水平轨迹误差，单位m
apParams.trajGenParams.aptRadiusNE_m_f = single(1.0);     % 水平航点接受半径，单位m，应该小于航点提前分界面判定的距离
apParams.trajGenParams.minLookaheadDist_m_f = single(5.0);     % 最小前视距离，单位m
apParams.trajGenParams.minSpdTurn_mps_f = single(0.2);     % 转弯判断最小轨迹速度，单位m/s
apParams.trajGenParams.cosThresholdTurn_f = single(0.98);    % 转弯判断方向余弦阈值
apParams.trajGenParams.fwdFixedSpd_mps_f = single(20);      % 保证顺利完成前倾，旋翼水平倾转转换速度，单位m/s

%% 旋翼垂向轨迹生成器参数
apParams.trajGenParams.maxJerkU_mps3_f = single(2.0);      % 旋翼垂向最大加加速度幅值，单位m/s^3
apParams.trajGenParams.maxTrajErrU_m_f = single(1.5);     % 最大允许垂向轨迹误差，单位m
apParams.trajGenParams.acceptRadiusU_m_f = single(0.8);     % 垂向航点接受半径，单位m

% 旋翼垂直起飞计算参数
apParams.trajGenParams.takeoffSwitchKp = single(0.5);          % 起飞时速度切换位置比例
apParams.trajGenParams.climbRate_mps_f = single(0.5);          % 起飞时爬升率，单位m/s
apParams.trajGenParams.maxSwitchAlt_m_f = single(20);          % 最大切换高度，单位m

% 旋翼下降率计算参数
apParams.trajGenParams.highHeight_m_f = single(10.0);       % 最大下降率对应的相对高度，单位m
apParams.trajGenParams.lowHeight_m_f = single(3.0);         % 最小下降率对应的相对高度，单位m
apParams.trajGenParams.maxDescentRate_mps_f = single(1.5);  % 最大下降率幅值，单位m/s
apParams.trajGenParams.minDescentRate_mps_f = single(0.5);  % 最小下降率幅值，单位m/s
apParams.trajGenParams.fixedDescentRate_mps_f = single(0.5);  % 固定下降率幅值，单位m/s

%% 旋翼位置导引参数
% 垂向位置导引
apParams.posGuiParams.kpU = single(0.1);       %垂向位置导引比例参数

% 水平位置导引
apParams.posGuiParams.kpN = single(0.1);       %N向位置导引比例参数
apParams.posGuiParams.kpE = single(0.1);       %E向位置导引比例参数
apParams.posGuiParams.switchSpdGuiHeight = single(0.3);       %相对高度到达此高度以下时，松开水平位置导引，速度给0

%% 旋翼偏航角导引参数
apParams.yawGuiParams.minYawFollowSpd_mps_f = single(0.1);  %轨迹水平速度低于该值时，不按照轨迹速度更新航向

%% 固定翼垂速导引参数
apParams.vuGuiParams.kpU = single(0.7);       %垂向位置反馈比例参数
apParams.vuGuiParams.switchLeadDist_m_f = single(10);    %直线航点提前分界面距离，m；
apParams.vuGuiParams.minHorizonLength_m_f = single(1.0);       %最小有效水平航段长度

%% 固定翼空速导引参数
apParams.airSpdGuiParams.cruiseAirSpd_mps_f = single(50);   %默认巡航空速，m/s
apParams.airSpdGuiParams.backTransDecel_mps2_f = single(1.8);    %后倾期望水平减速度，m/s^2
apParams.airSpdGuiParams.backTransStopRadius_m_f = single(10);  %后倾停车位置半径，m

%% 固定翼L1直线导引参数
apParams.l1GuiParams.kdLine_f = single(0.8);
apParams.l1GuiParams.ktLine_f = single(20);
apParams.l1GuiParams.minVxySpd = single(3);     % 最小允许速度

%% 固定翼L1圆弧导引参数
apParams.l1GuiParams.kdCir_f = single(0.8);
apParams.l1GuiParams.ktCir_f = single(20);
apParams.l1GuiParams.minCirRadius_m_f = single(200);        % 最小圆弧半径

%% 性能参数
%速度包线
%飞行包线应作为AP的输入，目前为测试旋翼前向速度参数
apParams.perfParams.maxVx_mps_f = single(60);       
apParams.perfParams.minVx_mps_f = single(-10);
apParams.perfParams.maxVy_mps_f = single(5);       %旋翼侧向速度参数
apParams.perfParams.minVy_mps_f = single(-5);
apParams.perfParams.maxVz_mps_f = single(3.0);        % 最大上升速度
apParams.perfParams.minVz_mps_f = single(-2.0);       % 最大下降速度对应的负值
%加速度包线
apParams.perfParams.maxAx_mps2_f = single(2.0);       % 最大前向加速度
apParams.perfParams.minAx_mps2_f = single(-2.0);      % 最大前向减速度
apParams.perfParams.maxAy_mps2_f = single(2.0);       % 最大右向加速度
apParams.perfParams.minAy_mps2_f = single(-2.0);      % 最大左向加速度
apParams.perfParams.maxAz_mps2_f = single(2.0);       % 最大向上加速度
apParams.perfParams.minAz_mps2_f = single(-2.0);      % 最大向下加速度
% 姿态角包线
apParams.perfParams.maxRollAngle_deg_f = single(30.0);
apParams.perfParams.minRollAngle_deg_f = single(-30.0);
apParams.perfParams.maxPitchAngle_deg_f = single(20.0);
apParams.perfParams.minPitchAngle_deg_f = single(-15.0);
% 角速度包线
apParams.perfParams.maxRollAngleRate_dps_f = single(20.0);
apParams.perfParams.minRollAngleRate_dps_f = single(-20.0);
apParams.perfParams.maxPitchAngleRate_dps_f = single(20.0);
apParams.perfParams.minPitchAngleRate_dps_f = single(-20.0);
apParams.perfParams.maxYawAngleRate_dps_f = single(15.0);
apParams.perfParams.minYawAngleRate_dps_f = single(-15.0);

%% 到达判定参数
% 悬停位置判定
apParams.hoverPosJudge.holdTimeCond_s_f = single(2.0);  % 判定持续时间
apParams.hoverPosJudge.spdNECond_mps_f = single(2.0);   % 水平速度判定条件
apParams.hoverPosJudge.distNECond_m_f = single(5.0);    % 水平位置判定条件
apParams.hoverPosJudge.distUCond_m_f = single(2.0);     % 高度判定条件

% 悬停航向对准判定
apParams.hoverYawJudge.holdTimeCond_s_f = single(2.0);  % 判定持续时间，悬停判定和偏航角判定持续时间
apParams.hoverYawJudge.spdNECond_mps_f = single(2.0);   % 水平速度判定条件
apParams.hoverYawJudge.distNECond_m_f = single(5.0);    % 水平位置判定条件
apParams.hoverYawJudge.distUCond_m_f = single(3.0);     % 高度判定条件
apParams.hoverYawJudge.yawTolerance_deg_f = single(5.0);     % 偏航角判定条件
apParams.hoverYawJudge.minYawDirDist_m_f = single(0.5);     % 能够计算偏航角时最小距离

% 旋翼直线到达判定
apParams.mcLineJudge.minJudgeDist_m_f = single(2);      % 旋翼提前分界面判定的L最小值
apParams.mcLineJudge.maxJudgeDist_m_f = single(8);     % 旋翼提前分界面判定的L最大值
apParams.mcLineJudge.segmentLimitRatio_f = single(0.45);      % 航段长度的比例系数

% 固定翼直线到达判定
apParams.fwLineJudge.minJudgeDist_m_f = single(20);      % 固定翼提前分界面判定的L最小值
apParams.fwLineJudge.maxJudgeDist_m_f = single(500);     % 固定翼提前分界面判定的L最大值
apParams.fwLineJudge.segmentLimitRatio_f = single(0.45);      % 航段长度的比例系数
apParams.fwLineJudge.effectRollRate_dps_f  = single(20);     % 固定翼有效滚转角速率
apParams.fwLineJudge.judgeDistMargin_f  = single(1.2);     % 提前距离裕度系数，必须不小于1
apParams.fwLineJudge.turnRollPlan_deg_f  = single(30);     % 使用目标转弯速度下允许滚转角，目前先给定一个角度

% 圆弧判定
apParams.cirJudge.cirExitTolerance_deg_f  = single(5);     % 角度容差
apParams.cirJudge.cirAltTolerance_m_f  = single(5);        % 高度容差

%% 任务管理-人工选择任务模式
apParams.selectMssnMode.fwdTiltRltHgt_m_f  = single(10);     % 自动前倾要求的相对高度
apParams.selectMssnMode.minLoiterHgt_m_f  = single(20);      % 定点盘旋要求的最低相对高度


%% 任务规划
% 自动前倾
apParams.fwdTiltPlan.fwdTiltDist_m_f = single(1500);      % 完全前倾距离
apParams.fwdTiltPlan.fwdTiltMinUseVel_mps_f = single(1);      % 大于这个速度需要判断速度与偏航角方向的一致性
apParams.fwdTiltPlan.fwdTiltYawLimit_deg_f = single(20);      % 速度与偏航角偏差

% 定向后倾
apParams.directBwdTiltPlan.bwdTiltDist_m_f = single(2000);      % 完全后倾距离

% 定点盘旋

% 航线飞行

% 定点着陆
apParams.pointLandPlan.fallbackBrakeAccNE_mps2_f = single(0.5);      % 水平加速度包线不可用时的备用刹车加速度
apParams.pointLandPlan.stopDistMarginNE_f = single(1.1);        % 水平刹停距离裕度，必须不小于1
apParams.pointLandPlan.fallbackBrakeAccU_mps2_f = single(0.5);      % 垂向加速度包线不可用时的备用刹车加速度
apParams.pointLandPlan.stopDistMarginU_f = single(1.1);        % 垂向刹停距离裕度，必须不小于1

% 返航
apParams.RTHPlan.rthTiltTranSpd_mps_f = single(30);         % 超过此速度固定翼返航，低于此速度先倾转再固定翼返航
apParams.RTHPlan.rthSetRoll_deg_f = single(30);         % 返航规划设定滚转角
apParams.RTHPlan.backFlightFarDist_m_f = single(150);   % 大于此距离采用悬停航向对准后返航，小于此距离锁定当前航向返航（航点指定）
apParams.RTHPlan.AllowOfSpdDirect_deg_f = single(5);   % 速度方向与进场方向误差角度
apParams.RTHPlan.AllowOfYawDirect_deg_f = single(5);   % 速度方向与路径方向误差角度
apParams.RTHPlan.mcFlightRegionRadius_m_f = single(1000);   % 小于这个水平范围，如果当前是旋翼构型则直接返航
apParams.RTHPlan.yawThdReturn_deg_f = single(10);      % 直接返航与刹停返航角度区分
apParams.RTHPlan.minHgtOfMCRth_m_f = single(4);        % 旋翼返航时的最低相对高度
apParams.RTHPlan.setHgtOfFWRth_m_f = single(10);        % 倾转/固定翼返航时的设定相对高度

% 自动后倾
apParams.bwdTiltPlan.minBackTiltHag_m_f = single(20);        % 最低允许相对高度，m
apParams.bwdTiltPlan.maxBackTiltHag_m_f = single(50);        % 最高允许相对高度，m
apParams.bwdTiltPlan.velCapAngle_deg_f = single(10);        % 速度方向捕获角，deg
apParams.bwdTiltPlan.approachCapAngle_deg_f = single(10);        % 进场方向捕获角，deg
apParams.bwdTiltPlan.capLeadTime_s_f = single(5);        % 捕获提前时间，s


Simulink.Parameter(apParams);