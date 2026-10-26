function [stopPos_deg_d, hoverAlt_m_f] = predictPointLandStopPoint(nav_struct, perfParams_struct, pointLandParam, isMcFlight_b)
%#codegen
%
% 定点着陆停止点预测
%
% 功能：
% 1. 根据当前水平地速预测水平刹停位置；
% 2. 水平速度较小时，使用当前偏航角作为刹停方向；
% 3. 根据机体系刹车方向和椭圆加速度包线计算水平刹车能力；
% 4. 根据当前垂向速度预测垂向刹停高度；
% 5. isMcFlight_b为false时，输出当前位置和当前高度。
%
% pointLandParam：
% 1：水平轨迹最大jerk，m/s^3；
% 2：水平备用刹车加速度，m/s^2；
% 3：水平刹停距离裕度，不小于1；
% 4：垂向轨迹最大jerk，m/s^3；
% 5：垂向备用刹车加速度，m/s^2；
% 6：垂向刹停距离裕度，不小于1。
%
% 输出：
% stopPos_deg_d  预测停止点经纬度，[Lat, Lon]，deg；
% hoverAlt_m_f   预测停止点绝对高度，m。

smallValue_f = single(0.001);
deg2rad_f = single(0.0174532925199433);

maxJerkNE_mps3_f = max(pointLandParam(1), smallValue_f);
fallbackBrakeAccNE_mps2_f = max(pointLandParam(2), smallValue_f);
stopDistMarginNE_f = max(pointLandParam(3), single(1.0));

maxJerkU_mps3_f = max(pointLandParam(4), smallValue_f);
fallbackBrakeAccU_mps2_f = max(pointLandParam(5), smallValue_f);
stopDistMarginU_f = max(pointLandParam(6), single(1.0));

%% 当前运动状态

velN_mps_f = nav_struct.velN_mps_f;
velE_mps_f = nav_struct.velE_mps_f;
velU_mps_f = nav_struct.velU_mps_f;

groundSpd_mps_f = sqrt(velN_mps_f * velN_mps_f + velE_mps_f * velE_mps_f);
absVelU_mps_f = abs(velU_mps_f);

%% 确定水平规划方向

yaw_rad_f = nav_struct.yaw_deg_f * deg2rad_f;

if groundSpd_mps_f > single(0.1)
    moveDirN_f = velN_mps_f / groundSpd_mps_f;
    moveDirE_f = velE_mps_f / groundSpd_mps_f;
else
    moveDirN_f = cos(yaw_rad_f);
    moveDirE_f = sin(yaw_rad_f);
end

%% 将水平刹车方向转换到机体系

brakeDirN_f = -moveDirN_f;
brakeDirE_f = -moveDirE_f;

cosYaw_f = cos(yaw_rad_f);
sinYaw_f = sin(yaw_rad_f);

brakeDirX_f = cosYaw_f * brakeDirN_f + sinYaw_f * brakeDirE_f;
brakeDirY_f = -sinYaw_f * brakeDirN_f + cosYaw_f * brakeDirE_f;

%% 根据刹车方向选择机体系各轴加速度能力

if brakeDirX_f >= single(0.0)
    brakeAccXLimit_mps2_f = max(perfParams_struct.maxAx_mps2_f, single(0.0));
else
    brakeAccXLimit_mps2_f = max(abs(perfParams_struct.minAx_mps2_f), single(0.0));
end

if brakeDirY_f >= single(0.0)
    brakeAccYLimit_mps2_f = max(perfParams_struct.maxAy_mps2_f, single(0.0));
else
    brakeAccYLimit_mps2_f = max(abs(perfParams_struct.minAy_mps2_f), single(0.0));
end

%% 计算当前方向上的水平刹车加速度

brakeAccInvSq_f = single(0.0);
brakeAccValid_b = true;

if abs(brakeDirX_f) > smallValue_f
    if brakeAccXLimit_mps2_f > smallValue_f
        brakeAccInvSq_f = brakeAccInvSq_f + brakeDirX_f * brakeDirX_f / (brakeAccXLimit_mps2_f * brakeAccXLimit_mps2_f);
    else
        brakeAccValid_b = false;
    end
end

if abs(brakeDirY_f) > smallValue_f
    if brakeAccYLimit_mps2_f > smallValue_f
        brakeAccInvSq_f = brakeAccInvSq_f + brakeDirY_f * brakeDirY_f / (brakeAccYLimit_mps2_f * brakeAccYLimit_mps2_f);
    else
        brakeAccValid_b = false;
    end
end

if brakeAccValid_b && brakeAccInvSq_f > smallValue_f
    brakeAccNE_mps2_f = single(1.0) / sqrt(brakeAccInvSq_f);
else
    brakeAccNE_mps2_f = fallbackBrakeAccNE_mps2_f;
end

%% 预测水平刹停位置偏移

stopOffsetNE_m_f = single(zeros(2, 1));

if isMcFlight_b && groundSpd_mps_f > smallValue_f
    stopDistNE_m_f = groundSpd_mps_f * groundSpd_mps_f / (single(2.0) * brakeAccNE_mps2_f);
    stopDistNE_m_f = stopDistNE_m_f + single(2.0) * brakeAccNE_mps2_f * groundSpd_mps_f / maxJerkNE_mps3_f;
    stopDistNE_m_f = stopDistNE_m_f * stopDistMarginNE_f;

    stopOffsetNE_m_f(1) = stopDistNE_m_f * moveDirN_f;
    stopOffsetNE_m_f(2) = stopDistNE_m_f * moveDirE_f;
end

%% 预测垂向刹停高度

hoverAlt_m_f = nav_struct.alt_m_f;

if isMcFlight_b && absVelU_mps_f > single(0.1)
    if velU_mps_f > single(0.0)
        brakeAccUEnvelope_mps2_f = max(abs(perfParams_struct.minAz_mps2_f), single(0.0));
    else
        brakeAccUEnvelope_mps2_f = max(perfParams_struct.maxAz_mps2_f, single(0.0));
    end

    if brakeAccUEnvelope_mps2_f > smallValue_f
        brakeAccU_mps2_f = brakeAccUEnvelope_mps2_f;
    else
        brakeAccU_mps2_f = fallbackBrakeAccU_mps2_f;
    end

    stopDistU_m_f = absVelU_mps_f * absVelU_mps_f / (single(2.0) * brakeAccU_mps2_f);
    stopDistU_m_f = stopDistU_m_f + single(2.0) * brakeAccU_mps2_f * absVelU_mps_f / maxJerkU_mps3_f;
    stopDistU_m_f = stopDistU_m_f * stopDistMarginU_f;

    stopOffsetU_m_f = sign(velU_mps_f) * stopDistU_m_f;
    hoverAlt_m_f = nav_struct.alt_m_f + stopOffsetU_m_f;
end

%% 将水平偏移转换为经纬度

referenceLoc_deg_d = [nav_struct.lat_deg_d, nav_struct.lon_deg_d];
stopPos_deg_d = offset(stopOffsetNE_m_f(1), stopOffsetNE_m_f(2), referenceLoc_deg_d);

end
