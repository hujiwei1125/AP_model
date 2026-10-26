function [nav_struct, posNEU_m_f, velNEU_mps_f, accNEU_mps2_f, attEuler_deg_f, rateBody_dps_f] = simple6DofPlant(apCmd_struct, flightPhase_enum, reset_b, dt_s_f, initState_f, refLat_deg_d, refLon_deg_d, ...
    refAlt_m_f, groundAlt_m_f)
%#codegen
%
% 面向AP逻辑和功能验证的简化六自由度运动学模型。
%
% 设计原则：
% 1. 直接使用AP已经完成限幅、限速处理的速度和姿态指令；
% 2. 不再增加速度、加速度、jerk和姿态响应参数；
% 3. 旋翼阶段直接跟踪机体系速度、垂速和姿态角指令；
% 4. 倾转/固定翼阶段仅保留滚转角产生协调转弯这一必要运动学关系；
% 5. 输出字段与AP_v8(3)中的apTransNavBus完全一致；
% 6. 本拍输出当前状态，再用当前指令更新下一拍状态，避免直接馈通代数环。
%
% flightPhase_enum：
% 0 INVALID，1 MC_GROUND，2 MC_FLIGHT，3 TILT_FLIGHT，4 FW_FLIGHT。
%
% initState_f，12x1：
% 1:3   局部NEU位置[N;E;U]，m；
% 4:6   NEU速度[Vn;Ve;Vu]，m/s；
% 7:9   姿态角[roll;pitch;yaw]，deg；
% 10:12 机体系角速度[p;q;r]，deg/s。

persistent posStateNEU_m_f
persistent velStateNEU_mps_f
persistent accStateNEU_mps2_f
persistent attState_deg_f
persistent rateState_dps_f
persistent init_b

if isempty(init_b)
    posStateNEU_m_f = single(zeros(3, 1));
    velStateNEU_mps_f = single(zeros(3, 1));
    accStateNEU_mps2_f = single(zeros(3, 1));
    attState_deg_f = single(zeros(3, 1));
    rateState_dps_f = single(zeros(3, 1));
    init_b = false;
end

loadInit_b = ~init_b || reset_b;

if loadInit_b
    posStateNEU_m_f = single(initState_f(1:3));
    velStateNEU_mps_f = single(initState_f(4:6));
    accStateNEU_mps2_f = single(zeros(3, 1));
    attState_deg_f = single(initState_f(7:9));
    attState_deg_f(3) = wrap180Single(attState_deg_f(3));
    rateState_dps_f = single(initState_f(10:12));
    init_b = true;
end

%% 输出当前状态

posNEU_m_f = posStateNEU_m_f;
velNEU_mps_f = velStateNEU_mps_f;
accNEU_mps2_f = accStateNEU_mps2_f;
attEuler_deg_f = attState_deg_f;
rateBody_dps_f = rateState_dps_f;

nav_struct = makeNavOutput(posStateNEU_m_f, velStateNEU_mps_f, accStateNEU_mps2_f, attState_deg_f, rateState_dps_f, refLat_deg_d, refLon_deg_d, refAlt_m_f, groundAlt_m_f);

% 复位拍只装载和输出初始状态。
if loadInit_b
    return;
end

% AP未运行时保持当前状态，清除运动学导数。
if ~logical(apCmd_struct.apRunState_b)
    accStateNEU_mps2_f = single(zeros(3, 1));
    rateState_dps_f = single(zeros(3, 1));
    return;
end

%% 飞行阶段

dtUse_s_f = max(single(dt_s_f), single(0.0001));
flightPhase_u8 = uint8(flightPhase_enum);

isMcPhase_b = flightPhase_u8 == uint8(1) || flightPhase_u8 == uint8(2);
isFwPhase_b = flightPhase_u8 == uint8(3) || flightPhase_u8 == uint8(4);

if ~isMcPhase_b && ~isFwPhase_b
    isMcPhase_b = true;
end

DEG2RAD_F = single(pi / 180.0);
RAD2DEG_F = single(180.0 / pi);
GRAVITY_MPS2_F = single(9.80665);
MIN_TURN_SPD_MPS_F = single(1.0);

%% 姿态更新

attNext_deg_f = attState_deg_f;
attNext_deg_f(1) = single(apCmd_struct.rollCmd_deg_f);
attNext_deg_f(2) = single(apCmd_struct.pitchCmd_deg_f);

if isMcPhase_b
    attNext_deg_f(3) = wrap180Single(single(apCmd_struct.yawCmd_deg_f));
else
    tasCmd_mps_f = max(single(apCmd_struct.vxCmd_mps_f), single(0.0));
    yawRate_dps_f = single(0.0);

    if tasCmd_mps_f > MIN_TURN_SPD_MPS_F
        rollCmd_rad_f = attNext_deg_f(1) * DEG2RAD_F;
        yawRate_dps_f = GRAVITY_MPS2_F * tan(rollCmd_rad_f) / tasCmd_mps_f * RAD2DEG_F;
    end

    attNext_deg_f(3) = wrap180Single(attState_deg_f(3) + yawRate_dps_f * dtUse_s_f);
end

%% 速度更新

yawNext_rad_f = attNext_deg_f(3) * DEG2RAD_F;
cosYaw_f = cos(yawNext_rad_f);
sinYaw_f = sin(yawNext_rad_f);

velNextNEU_mps_f = single(zeros(3, 1));

if isMcPhase_b
    vxCmd_mps_f = single(apCmd_struct.vxCmd_mps_f);
    vyCmd_mps_f = single(apCmd_struct.vyCmd_mps_f);

    velNextNEU_mps_f(1) = cosYaw_f * vxCmd_mps_f - sinYaw_f * vyCmd_mps_f;
    velNextNEU_mps_f(2) = sinYaw_f * vxCmd_mps_f + cosYaw_f * vyCmd_mps_f;
    velNextNEU_mps_f(3) = single(apCmd_struct.vuCmd_mps_f);
else
    tasCmd_mps_f = max(single(apCmd_struct.vxCmd_mps_f), single(0.0));
    vuCmd_mps_f = single(apCmd_struct.vuCmd_mps_f);
    horizonSpdCmd_mps_f = sqrt(max(tasCmd_mps_f * tasCmd_mps_f - vuCmd_mps_f * vuCmd_mps_f, single(0.0)));

    velNextNEU_mps_f(1) = horizonSpdCmd_mps_f * cosYaw_f;
    velNextNEU_mps_f(2) = horizonSpdCmd_mps_f * sinYaw_f;
    velNextNEU_mps_f(3) = vuCmd_mps_f;
end

%% 由相邻状态计算加速度和机体系角速度

accNextNEU_mps2_f = (velNextNEU_mps_f - velStateNEU_mps_f) / dtUse_s_f;

eulerRate_dps_f = single(zeros(3, 1));
eulerRate_dps_f(1) = (attNext_deg_f(1) - attState_deg_f(1)) / dtUse_s_f;
eulerRate_dps_f(2) = (attNext_deg_f(2) - attState_deg_f(2)) / dtUse_s_f;
eulerRate_dps_f(3) = wrap180Single(attNext_deg_f(3) - attState_deg_f(3)) / dtUse_s_f;

roll_rad_f = attState_deg_f(1) * DEG2RAD_F;
pitch_rad_f = attState_deg_f(2) * DEG2RAD_F;

rateNext_dps_f = single(zeros(3, 1));
rateNext_dps_f(1) = eulerRate_dps_f(1) - eulerRate_dps_f(3) * sin(pitch_rad_f);
rateNext_dps_f(2) = eulerRate_dps_f(2) * cos(roll_rad_f) + eulerRate_dps_f(3) * sin(roll_rad_f) * cos(pitch_rad_f);
rateNext_dps_f(3) = -eulerRate_dps_f(2) * sin(roll_rad_f) + eulerRate_dps_f(3) * cos(roll_rad_f) * cos(pitch_rad_f);

%% 位置积分

posNextNEU_m_f = posStateNEU_m_f + single(0.5) * (velStateNEU_mps_f + velNextNEU_mps_f) * dtUse_s_f;

altNext_m_f = single(refAlt_m_f) + posNextNEU_m_f(3);

if altNext_m_f < single(groundAlt_m_f)
    posNextNEU_m_f(3) = single(groundAlt_m_f) - single(refAlt_m_f);

    if velNextNEU_mps_f(3) < single(0.0)
        velNextNEU_mps_f(3) = single(0.0);
        accNextNEU_mps2_f(3) = single(0.0);
    end
end

%% 保存下一拍状态

posStateNEU_m_f = posNextNEU_m_f;
velStateNEU_mps_f = velNextNEU_mps_f;
accStateNEU_mps2_f = accNextNEU_mps2_f;
attState_deg_f = attNext_deg_f;
rateState_dps_f = rateNext_dps_f;

end


function nav_struct = makeNavOutput(posNEU_m_f, velNEU_mps_f, accNEU_mps2_f, att_deg_f, rate_dps_f, refLat_deg_d, refLon_deg_d, refAlt_m_f, groundAlt_m_f)
%#codegen

EARTH_RADIUS_M_D = 6371000.0;
RAD2DEG_D = 180.0 / pi;

lat_deg_d = double(refLat_deg_d) + double(posNEU_m_f(1)) / EARTH_RADIUS_M_D * RAD2DEG_D;
longitudeScale_d = max(cos(double(refLat_deg_d) * pi / 180.0), 0.01);
lon_deg_d = double(refLon_deg_d) + double(posNEU_m_f(2)) / (EARTH_RADIUS_M_D * longitudeScale_d) * RAD2DEG_D;
lon_deg_d = mod(lon_deg_d + 180.0, 360.0) - 180.0;

alt_m_f = single(refAlt_m_f) + posNEU_m_f(3);
hag_m_f = max(alt_m_f - single(groundAlt_m_f), single(0.0));
vg_mps_f = single(hypot(double(velNEU_mps_f(1)), double(velNEU_mps_f(2))));
tas_mps_f = single(sqrt(double(velNEU_mps_f(1) * velNEU_mps_f(1) + velNEU_mps_f(2) * velNEU_mps_f(2) + velNEU_mps_f(3) * velNEU_mps_f(3))));

nav_struct = struct('lat_deg_d', lat_deg_d, 'lon_deg_d', lon_deg_d, 'lonlatValid_b', true, 'alt_m_f', alt_m_f, 'altValid_b', true, 'velN_mps_f', velNEU_mps_f(1), 'velE_mps_f', velNEU_mps_f(2), 'velNEValid_b', true, 'velU_mps_f', velNEU_mps_f(3), 'velUValid_b', true, 'accN_mps2_f', accNEU_mps2_f(1), 'accE_mps2_f', accNEU_mps2_f(2), 'accU_mps2_f', accNEU_mps2_f(3), 'roll_deg_f', att_deg_f(1), 'pitch_deg_f', att_deg_f(2), 'attValid_b', true, 'yaw_deg_f', wrap180Single(att_deg_f(3)), 'yawValid_b', true, 'p_dps_f', rate_dps_f(1), 'q_dps_f', rate_dps_f(2), 'r_dps_f', rate_dps_f(3), 'hag_m_f', hag_m_f, 'hagValid_b', true, 'vg_mps_f', vg_mps_f, 'vgValid_b', true, 'tas_mps_f', tas_mps_f, 'tasValid_b', true);

end


function angle_deg_f = wrap180Single(angle_deg_f)
%#codegen

angle_deg_f = mod(angle_deg_f + single(180.0), single(360.0)) - single(180.0);

end
