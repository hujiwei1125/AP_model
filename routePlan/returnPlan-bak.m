function [returnRoute_struct, validNum_u8, routePlanValid_b] = returnPlan(rthPoint_struct, flightPhase_enum, RTHParam, perfParams_struct, pointLandParam, nav_struct)
%#codegen
%
% 返航航路规划
%
% returnPlanType_u8：
% 0：无返航规划
% 1：固定翼返航
% 2：旋翼/低速倾转先前倾，再按固定翼返航
% 3：旋翼直接返航
% 4：旋翼地面等待返航着陆
%
% 返航类型选择原则：
%
% 1. MC_GROUND：
%    原地执行旋翼着陆。
%
% 2. FW_FLIGHT / 高速TILT_FLIGHT：
%    采用固定翼返航。
%
% 3. MC_FLIGHT / 低速TILT_FLIGHT：
%    若位于旋翼返航范围内，采用旋翼返航；
%    若剩余距离不足以完成“完整前倾 + 完整后倾”，也采用旋翼返航；
%    只有剩余距离足够时才采用MC->FW->MC全剖面返航。
%
% 4. 全剖面返航：
%    directPass_b=true  ：直线过渡
%    directPass_b=false ：圆弧过渡
%
% 注意：
% fullFwdTiltDist_m_f、fullBwdTiltDist_m_f均按完整构型转换距离使用。
% 当前没有单独的固定翼稳定飞行最小距离参数。

%% 初始化

maxNum_u8 = uint8(30);

routePoint_struct = struct('wpType_u8', WP_TYPE.INVALID_WP, 'lat_deg_d', double(0), 'lon_deg_d', double(0), 'alt_m_f', single(0), 'spd_mps_f', single(0), 'radius_m_f', single(0), 'fastWp_b', false, 'yawType_u8', uint8(0), 'yaw_deg_f', nav_struct.yaw_deg_f, 'timeOrCount_u16', uint16(0), 'arrivalCond_u8', uint8(0));

returnRoute_struct = repmat(routePoint_struct, maxNum_u8, 1);

validNum_u8 = uint8(0);
routePlanValid_b = false;

%% 当前导航状态

curLat_deg_d = nav_struct.lat_deg_d;
curLon_deg_d = nav_struct.lon_deg_d;
curAlt_m_f = nav_struct.alt_m_f;
curYaw_deg_f = nav_struct.yaw_deg_f;

curVelNEU_mps_f = [nav_struct.velN_mps_f, nav_struct.velE_mps_f, nav_struct.velU_mps_f];

%% 返航参数

G_mps2_f = RTHParam(1);
rthTiltTranSpd_mps_f = RTHParam(2);
fullFwdTiltDist_m_f = RTHParam(3);
fullBwdTiltDist_m_f = RTHParam(4);
rthSetRoll_deg_f = RTHParam(5);
cruiseSpd_mps_f = RTHParam(6);
backFlightFarDist_m_f = RTHParam(7);
AllowOfSpdDirect_deg_f = RTHParam(8);
AllowOfYawDirect_deg_f = RTHParam(9);
mcFlightRegionRadius_m_f = RTHParam(10);
yawThdReturn_deg_f = RTHParam(11);
minHgtOfMCRth_m_f = RTHParam(12);
setHgtOfFWRth_m_f = RTHParam(13);

%% 返航点

rthEntryDirect_deg_f = rthPoint_struct.yaw_deg_f;
rthPointLat_deg_d = rthPoint_struct.lat_deg_d;
rthPointLon_deg_d = rthPoint_struct.lon_deg_d;

% rthPoint_struct.alt_m_f定义为返航点地面绝对高度。
rthGroundAlt_m_f = rthPoint_struct.alt_m_f;

%% 返航高度

% minHgtOfMCRth_m_f为相对返航点地面的旋翼最低返航高度。
mcRthMinAlt_m_f = rthGroundAlt_m_f + max(minHgtOfMCRth_m_f, single(0.0));

% setHgtOfFWRth_m_f为相对返航点地面的固定翼返航高度。
fwRthAlt_m_f = rthGroundAlt_m_f + max(setHgtOfFWRth_m_f, single(0.0));

% 当前没有独立的后倾点高度输入。
bwdTiltAlt_m_f = curAlt_m_f;

%% 基本阈值

homePosTolerance_m_f = single(0.1);

%% 当前水平速度

horizontalSpd_mps_f = single(hypot(double(curVelNEU_mps_f(1)), double(curVelNEU_mps_f(2))));
curHorizontalVelNEU_mps_f = [curVelNEU_mps_f(1), curVelNEU_mps_f(2), single(0.0)];

%% 当前构型

tiltFast_b = flightPhase_enum == FLIGHT_PHASE.TILT_FLIGHT && horizontalSpd_mps_f >= rthTiltTranSpd_mps_f;
tiltSlow_b = flightPhase_enum == FLIGHT_PHASE.TILT_FLIGHT && horizontalSpd_mps_f < rthTiltTranSpd_mps_f;

fwReturn_b = flightPhase_enum == FLIGHT_PHASE.FW_FLIGHT || tiltFast_b;
rotorToFwReturn_b = flightPhase_enum == FLIGHT_PHASE.MC_FLIGHT || tiltSlow_b;

%% 当前位置至Home水平距离

curToHomeNE_m_f = getDistanceNE([curLat_deg_d, curLon_deg_d], [rthPointLat_deg_d, rthPointLon_deg_d]);
distCur2Home_m_f = single(norm(curToHomeNE_m_f));

%% 全剖面返航距离有效性

% 当前至少要求能够放下：
%
% 完整前倾距离 + 完整后倾距离

minFullProfileRthDist_m_f = max(fullFwdTiltDist_m_f, single(0.0)) + max(fullBwdTiltDist_m_f, single(0.0));

% 等于临界距离时，完全前倾点和后倾开始点刚好重合，
% 没有固定翼稳定飞行段，因此使用严格大于。
fullProfileDistValid_b = distCur2Home_m_f > minFullProfileRthDist_m_f;

%% 返航类型选择

returnPlanType_u8 = uint8(0);

if flightPhase_enum == FLIGHT_PHASE.MC_GROUND

    %% 旋翼地面
    returnPlanType_u8 = uint8(4);

elseif fwReturn_b

    %% 固定翼或高速倾转
    returnPlanType_u8 = uint8(1);

elseif rotorToFwReturn_b

    %% 旋翼或低速倾转

    % 已进入旋翼返航区域，或者剩余距离不足以完成全剖面返航，
    % 都直接采用旋翼返航。
    if distCur2Home_m_f <= mcFlightRegionRadius_m_f || ~fullProfileDistValid_b

        returnPlanType_u8 = uint8(3);

    else

        returnPlanType_u8 = uint8(2);

    end

end

%% 航路规划

switch returnPlanType_u8

    case uint8(4)

        %% 旋翼地面等待返航着陆

        returnRoute_struct(1).wpType_u8 = WP_TYPE.MC_LAND_WP;
        returnRoute_struct(1).lat_deg_d = curLat_deg_d;
        returnRoute_struct(1).lon_deg_d = curLon_deg_d;
        returnRoute_struct(1).alt_m_f = curAlt_m_f;
        returnRoute_struct(1).yaw_deg_f = curYaw_deg_f;
        returnRoute_struct(1).yawType_u8 = uint8(1);

        % 第1点作为锚点，第2点作为首个执行点。
        returnRoute_struct(2) = returnRoute_struct(1);

        validNum_u8 = uint8(2);
        routePlanValid_b = true;

    case uint8(3)

        %% 旋翼直接返航

        %% 当前状态锚点

        returnRoute_struct(1).wpType_u8 = WP_TYPE.MC_LINE_WP;
        returnRoute_struct(1).lat_deg_d = curLat_deg_d;
        returnRoute_struct(1).lon_deg_d = curLon_deg_d;
        returnRoute_struct(1).alt_m_f = curAlt_m_f;
        returnRoute_struct(1).yaw_deg_f = curYaw_deg_f;
        returnRoute_struct(1).spd_mps_f = horizontalSpd_mps_f;
        returnRoute_struct(1).yawType_u8 = uint8(0);
        returnRoute_struct(1).fastWp_b = true;

        %% 当前运动方向

        if horizontalSpd_mps_f < single(0.1)

            yawVel_deg_f = cldWrap180(curYaw_deg_f);

        else

            yawVel_deg_f = cldWrap180(atan2(curVelNEU_mps_f(2), curVelNEU_mps_f(1)) * single(180 / pi));

        end

        %% 当前是否已经位于Home上方

        homeReached_b = distCur2Home_m_f <= homePosTolerance_m_f;

        if homeReached_b

            deltaYaw_deg_f = single(0.0);

        else

            bearing_deg_f = getBearing([curLat_deg_d, curLon_deg_d], [rthPointLat_deg_d, rthPointLon_deg_d]);
            yawPos_deg_f = cldWrap180(bearing_deg_f);
            deltaYaw_deg_f = abs(cldWrap180(yawVel_deg_f - yawPos_deg_f));

        end

        %% 当前速度方向已经指向Home

        if homeReached_b || deltaYaw_deg_f < abs(yawThdReturn_deg_f)

            if homeReached_b

                returnAlt_m_f = curAlt_m_f;

            else

                returnAlt_m_f = max(curAlt_m_f, mcRthMinAlt_m_f);

            end

            %% Home上方进场点

            returnRoute_struct(2).wpType_u8 = WP_TYPE.MC_LAND_WP;
            returnRoute_struct(2).lat_deg_d = rthPointLat_deg_d;
            returnRoute_struct(2).lon_deg_d = rthPointLon_deg_d;
            returnRoute_struct(2).alt_m_f = returnAlt_m_f;
            returnRoute_struct(2).yaw_deg_f = curYaw_deg_f;
            returnRoute_struct(2).spd_mps_f = single(0.0);
            returnRoute_struct(2).yawType_u8 = uint8(2);

            %% 地面点

            returnRoute_struct(3) = returnRoute_struct(2);
            returnRoute_struct(3).alt_m_f = rthGroundAlt_m_f;

            validNum_u8 = uint8(3);

        else

            %% 当前速度方向没有指向Home，先刹停

            isMcFlight_b = true;

            [stopPos_deg_d, hoverAlt_m_f] = predictPointLandStopPoint(nav_struct, perfParams_struct, pointLandParam, isMcFlight_b);

            predictLat_deg_d = stopPos_deg_d(1);
            predictLon_deg_d = stopPos_deg_d(2);
            predictAlt_m_f = hoverAlt_m_f;

            predToHomeNE_m_f = getDistanceNE([predictLat_deg_d, predictLon_deg_d], [rthPointLat_deg_d, rthPointLon_deg_d]);
            distPred2Home_m_f = single(norm(predToHomeNE_m_f));

            %% 预测停止点

            returnRoute_struct(2).wpType_u8 = WP_TYPE.MC_LINE_WP;
            returnRoute_struct(2).lat_deg_d = predictLat_deg_d;
            returnRoute_struct(2).lon_deg_d = predictLon_deg_d;
            returnRoute_struct(2).alt_m_f = predictAlt_m_f;
            returnRoute_struct(2).yaw_deg_f = curYaw_deg_f;
            returnRoute_struct(2).yawType_u8 = uint8(1);
            returnRoute_struct(2).fastWp_b = false;

            %% 是否需要先原地爬升

            predictAtHome_b = distPred2Home_m_f <= homePosTolerance_m_f;
            needClimb_b = ~predictAtHome_b && predictAlt_m_f < mcRthMinAlt_m_f;

            if needClimb_b

                returnRoute_struct(2).yawType_u8 = uint8(1);

                returnRoute_struct(3) = returnRoute_struct(2);
                returnRoute_struct(3).alt_m_f = mcRthMinAlt_m_f;
                returnRoute_struct(3).yawType_u8 = uint8(1);

                landIdx_u8 = uint8(4);
                preLandIdx_u8 = uint8(3);
                landAlt_m_f = mcRthMinAlt_m_f;

            else

                landIdx_u8 = uint8(3);
                preLandIdx_u8 = uint8(2);
                landAlt_m_f = predictAlt_m_f;

            end

            %% Home上方进场点

            returnRoute_struct(landIdx_u8).wpType_u8 = WP_TYPE.MC_LAND_WP;
            returnRoute_struct(landIdx_u8).lat_deg_d = rthPointLat_deg_d;
            returnRoute_struct(landIdx_u8).lon_deg_d = rthPointLon_deg_d;
            returnRoute_struct(landIdx_u8).alt_m_f = landAlt_m_f;

            if distPred2Home_m_f >= backFlightFarDist_m_f

                returnRoute_struct(landIdx_u8).yawType_u8 = uint8(2);

                bearing_deg_f = getBearing([returnRoute_struct(preLandIdx_u8).lat_deg_d, returnRoute_struct(preLandIdx_u8).lon_deg_d], ...
                    [returnRoute_struct(landIdx_u8).lat_deg_d, returnRoute_struct(landIdx_u8).lon_deg_d]);

                returnRoute_struct(landIdx_u8).yaw_deg_f = cldWrap180(bearing_deg_f);

            else

                returnRoute_struct(landIdx_u8).yawType_u8 = uint8(1);
                returnRoute_struct(landIdx_u8).yaw_deg_f = curYaw_deg_f;

            end

            %% 地面点

            groundIdx_u8 = landIdx_u8 + uint8(1);

            returnRoute_struct(groundIdx_u8) = returnRoute_struct(landIdx_u8);
            returnRoute_struct(groundIdx_u8).alt_m_f = rthGroundAlt_m_f;

            validNum_u8 = groundIdx_u8;

        end

        routePlanValid_b = true;

    case uint8(1)

        %% 固定翼返航

        %% 当前飞行方向

        if horizontalSpd_mps_f >= single(0.1)

            curHorizonSpdDirect_deg_f = atan2(curVelNEU_mps_f(2), curVelNEU_mps_f(1)) * single(180 / pi);

        else

            curHorizonSpdDirect_deg_f = curYaw_deg_f;

        end

        curHorizonSpdDirect_deg_f = cldWrap180(curHorizonSpdDirect_deg_f);

        %% 后倾点

        bearing_return_bwd_deg_f = cldWrap180(rthEntryDirect_deg_f + single(180.0));

        bwd_tilt_point = offsetBearing( ...
            bearing_return_bwd_deg_f, ...
            fullBwdTiltDist_m_f, ...
            [rthPointLat_deg_d, rthPointLon_deg_d]);

        %% 当前点至后倾点

        cur_return_ne = getDistanceNE([curLat_deg_d, curLon_deg_d], [bwd_tilt_point(1), bwd_tilt_point(2)]);
        curReturnDist_m_f = single(norm(cur_return_ne));

        if curReturnDist_m_f <= single(0.1)

            pathDirect_deg_f = rthEntryDirect_deg_f;

        else

            pathDirect_deg_f = getBearing([curLat_deg_d, curLon_deg_d], [bwd_tilt_point(1), bwd_tilt_point(2)]);

        end

        %% 直线通过条件

        spdDirectErr_deg_f = abs(cldWrap180(curHorizonSpdDirect_deg_f - rthEntryDirect_deg_f));
        pathDirectErr_deg_f = abs(cldWrap180(curHorizonSpdDirect_deg_f - pathDirect_deg_f));

        directPass_b = spdDirectErr_deg_f < abs(AllowOfSpdDirect_deg_f) && pathDirectErr_deg_f < abs(AllowOfYawDirect_deg_f);

        %% 当前点至Home

        distCur2rthPoint = getDistanceNE([curLat_deg_d, curLon_deg_d], [rthPointLat_deg_d, rthPointLon_deg_d]);

        if directPass_b && norm(distCur2rthPoint) > fullBwdTiltDist_m_f

            %% 固定翼直线返航

            returnRoute_struct(1).wpType_u8 = WP_TYPE.FW_LINE_WP;
            returnRoute_struct(1).lat_deg_d = curLat_deg_d;
            returnRoute_struct(1).lon_deg_d = curLon_deg_d;
            returnRoute_struct(1).alt_m_f = curAlt_m_f;
            returnRoute_struct(1).yaw_deg_f = curYaw_deg_f;
            returnRoute_struct(1).spd_mps_f = horizontalSpd_mps_f;

            returnRoute_struct(2).wpType_u8 = WP_TYPE.FW_LINE_WP;
            returnRoute_struct(2).lat_deg_d = bwd_tilt_point(1);
            returnRoute_struct(2).lon_deg_d = bwd_tilt_point(2);
            returnRoute_struct(2).alt_m_f = fwRthAlt_m_f;
            returnRoute_struct(2).yaw_deg_f = rthEntryDirect_deg_f;
            returnRoute_struct(2).spd_mps_f = horizontalSpd_mps_f;

            returnRoute_struct(3).wpType_u8 = WP_TYPE.MC_LAND_WP;
            returnRoute_struct(3).lat_deg_d = rthPointLat_deg_d;
            returnRoute_struct(3).lon_deg_d = rthPointLon_deg_d;
            returnRoute_struct(3).alt_m_f = fwRthAlt_m_f;
            returnRoute_struct(3).yaw_deg_f = rthEntryDirect_deg_f;
            returnRoute_struct(3).spd_mps_f = single(0.0);
            returnRoute_struct(3).yawType_u8 = uint8(1);

            returnRoute_struct(4) = returnRoute_struct(3);
            returnRoute_struct(4).alt_m_f = rthGroundAlt_m_f;

            validNum_u8 = uint8(4);
            routePlanValid_b = true;

        else

            %% 固定翼圆弧返航

            trans_mode_u8 = circleTransMode( ...
                [curLat_deg_d, curLon_deg_d], ...
                [bwd_tilt_point(1), bwd_tilt_point(2)], ...
                curHorizonSpdDirect_deg_f, ...
                rthEntryDirect_deg_f);

            p1 = struct('lat_deg_d', double(curLat_deg_d), 'lon_deg_d', double(curLon_deg_d), 'alt_m_f', single(curAlt_m_f), 'yaw_deg_f', single(curHorizonSpdDirect_deg_f));

            p2 = struct('lat_deg_d', double(bwd_tilt_point(1)), 'lon_deg_d', double(bwd_tilt_point(2)), 'alt_m_f', bwdTiltAlt_m_f, 'yaw_deg_f', single(rthEntryDirect_deg_f));

            maxCircleNum_u8 = maxNum_u8 - uint8(2);

            if trans_mode_u8 == uint8(1) || trans_mode_u8 == uint8(2)

                if trans_mode_u8 == uint8(1)
                    rotateAngle_rad_f = single(pi / 2);
                else
                    rotateAngle_rad_f = single(-pi / 2);
                end

                [plan_result, num_u8] = sameRotationTransHandle( ...
                    p1, p2, rthSetRoll_deg_f, rotateAngle_rad_f, maxCircleNum_u8, ...
                    routePoint_struct, cruiseSpd_mps_f, G_mps2_f, curHorizontalVelNEU_mps_f);

            else

                if trans_mode_u8 == uint8(3)
                    rotateAngle_rad_f = single(pi / 2);
                else
                    rotateAngle_rad_f = single(-pi / 2);
                end

                [plan_result, num_u8] = changeRotationTransHandle( ...
                    p1, p2, rthSetRoll_deg_f, rotateAngle_rad_f, maxCircleNum_u8, ...
                    routePoint_struct, trans_mode_u8, cruiseSpd_mps_f, G_mps2_f, curHorizontalVelNEU_mps_f);

            end

            if num_u8 > uint8(0) && num_u8 <= maxCircleNum_u8

                num_d = double(num_u8);

                returnRoute_struct(1:num_d) = plan_result(1:num_d);

                landIdx_u8 = num_u8 + uint8(1);

                returnRoute_struct(landIdx_u8).wpType_u8 = WP_TYPE.MC_LAND_WP;
                returnRoute_struct(landIdx_u8).lat_deg_d = rthPointLat_deg_d;
                returnRoute_struct(landIdx_u8).lon_deg_d = rthPointLon_deg_d;
                returnRoute_struct(landIdx_u8).alt_m_f = fwRthAlt_m_f;
                returnRoute_struct(landIdx_u8).yaw_deg_f = rthEntryDirect_deg_f;
                returnRoute_struct(landIdx_u8).spd_mps_f = single(0.0);
                returnRoute_struct(landIdx_u8).yawType_u8 = uint8(2);

                groundIdx_u8 = landIdx_u8 + uint8(1);

                returnRoute_struct(groundIdx_u8) = returnRoute_struct(landIdx_u8);
                returnRoute_struct(groundIdx_u8).alt_m_f = rthGroundAlt_m_f;

                validNum_u8 = groundIdx_u8;
                routePlanValid_b = true;

            end

        end

    case uint8(2)

        %% 旋翼/低速倾转 -> 固定翼 -> 旋翼返航

        %% 当前飞行方向

        if horizontalSpd_mps_f >= single(0.1)

            curHorizonSpdDirect_deg_f = atan2(curVelNEU_mps_f(2), curVelNEU_mps_f(1)) * single(180 / pi);

        else

            curHorizonSpdDirect_deg_f = curYaw_deg_f;

        end

        curHorizonSpdDirect_deg_f = cldWrap180(curHorizonSpdDirect_deg_f);

        %% 完全前倾点

        fwd_tilt_point = offsetBearing( ...
            curHorizonSpdDirect_deg_f, ...
            fullFwdTiltDist_m_f, ...
            [curLat_deg_d, curLon_deg_d]);

        %% 后倾点

        bearing_return_bwd_deg_f = cldWrap180(rthEntryDirect_deg_f + single(180.0));

        bwd_tilt_point = offsetBearing( ...
            bearing_return_bwd_deg_f, ...
            fullBwdTiltDist_m_f, ...
            [rthPointLat_deg_d, rthPointLon_deg_d]);

        %% 当前点至后倾点

        cur_return_ne = getDistanceNE([curLat_deg_d, curLon_deg_d], [bwd_tilt_point(1), bwd_tilt_point(2)]);
        curReturnDist_m_f = single(norm(cur_return_ne));

        if curReturnDist_m_f <= single(0.1)

            pathDirect_deg_f = rthEntryDirect_deg_f;

        else

            pathDirect_deg_f = getBearing([curLat_deg_d, curLon_deg_d], [bwd_tilt_point(1), bwd_tilt_point(2)]);

        end

        %% 直线通过方向条件

        spdDirectErr_deg_f = abs(cldWrap180(curHorizonSpdDirect_deg_f - rthEntryDirect_deg_f));
        pathDirectErr_deg_f = abs(cldWrap180(curHorizonSpdDirect_deg_f - pathDirect_deg_f));

        directPass_b = spdDirectErr_deg_f < abs(AllowOfSpdDirect_deg_f) && pathDirectErr_deg_f < abs(AllowOfYawDirect_deg_f);

        %% 当前锚点类型

        if flightPhase_enum == FLIGHT_PHASE.MC_FLIGHT
            firstWpType_u8 = WP_TYPE.MC_LINE_WP;
        else
            firstWpType_u8 = WP_TYPE.FW_LINE_WP;
        end

        %% 全剖面内部路径形式选择
        %
        % 能进入case 2已经说明：
        %
        % distCur2Home >
        % fullFwdTiltDist + fullBwdTiltDist
        %
        % 所以此处只根据directPass_b选择直线或圆弧。

        if directPass_b

            %% 直线全剖面返航

            returnRoute_struct(1).wpType_u8 = firstWpType_u8;
            returnRoute_struct(1).lat_deg_d = curLat_deg_d;
            returnRoute_struct(1).lon_deg_d = curLon_deg_d;
            returnRoute_struct(1).alt_m_f = curAlt_m_f;
            returnRoute_struct(1).yaw_deg_f = curYaw_deg_f;
            returnRoute_struct(1).spd_mps_f = cruiseSpd_mps_f;

            %% 完全前倾点

            returnRoute_struct(2).wpType_u8 = WP_TYPE.FW_LINE_WP;
            returnRoute_struct(2).lat_deg_d = fwd_tilt_point(1);
            returnRoute_struct(2).lon_deg_d = fwd_tilt_point(2);
            returnRoute_struct(2).alt_m_f = curAlt_m_f;
            returnRoute_struct(2).yaw_deg_f = curHorizonSpdDirect_deg_f;
            returnRoute_struct(2).spd_mps_f = cruiseSpd_mps_f;
            returnRoute_struct(2).yawType_u8 = uint8(2);

            %% 后倾开始点

            returnRoute_struct(3).wpType_u8 = WP_TYPE.FW_LINE_WP;
            returnRoute_struct(3).lat_deg_d = bwd_tilt_point(1);
            returnRoute_struct(3).lon_deg_d = bwd_tilt_point(2);
            returnRoute_struct(3).alt_m_f = fwRthAlt_m_f;
            returnRoute_struct(3).yaw_deg_f = rthEntryDirect_deg_f;
            returnRoute_struct(3).spd_mps_f = cruiseSpd_mps_f;
            returnRoute_struct(3).yawType_u8 = uint8(2);

            %% Home上方旋翼降落点

            returnRoute_struct(4).wpType_u8 = WP_TYPE.MC_LAND_WP;
            returnRoute_struct(4).lat_deg_d = rthPointLat_deg_d;
            returnRoute_struct(4).lon_deg_d = rthPointLon_deg_d;
            returnRoute_struct(4).alt_m_f = fwRthAlt_m_f;
            returnRoute_struct(4).yaw_deg_f = rthEntryDirect_deg_f;
            returnRoute_struct(4).spd_mps_f = single(0.0);
            returnRoute_struct(4).yawType_u8 = uint8(2);

            %% 地面点

            returnRoute_struct(5) = returnRoute_struct(4);
            returnRoute_struct(5).alt_m_f = rthGroundAlt_m_f;

            validNum_u8 = uint8(5);
            routePlanValid_b = true;

        else

            %% 圆弧全剖面返航

            % 圆弧规划从完全前倾点开始。
            trans_mode_u8 = circleTransMode( ...
                [fwd_tilt_point(1), fwd_tilt_point(2)], ...
                [bwd_tilt_point(1), bwd_tilt_point(2)], ...
                curHorizonSpdDirect_deg_f, ...
                rthEntryDirect_deg_f);

            p1 = struct('lat_deg_d', double(fwd_tilt_point(1)), 'lon_deg_d', double(fwd_tilt_point(2)), 'alt_m_f', single(curAlt_m_f), 'yaw_deg_f', single(curHorizonSpdDirect_deg_f));

            p2 = struct('lat_deg_d', double(bwd_tilt_point(1)), 'lon_deg_d', double(bwd_tilt_point(2)), 'alt_m_f', bwdTiltAlt_m_f, 'yaw_deg_f', single(rthEntryDirect_deg_f));

            maxCircleNum_u8 = maxNum_u8 - uint8(3);

            if trans_mode_u8 == uint8(1) || trans_mode_u8 == uint8(2)

                if trans_mode_u8 == uint8(1)
                    rotateAngle_rad_f = single(pi / 2);
                else
                    rotateAngle_rad_f = single(-pi / 2);
                end

                [plan_result, num_u8] = sameRotationTransHandle( ...
                    p1, p2, rthSetRoll_deg_f, rotateAngle_rad_f, maxCircleNum_u8, ...
                    routePoint_struct, cruiseSpd_mps_f, G_mps2_f, curHorizontalVelNEU_mps_f);

            else

                if trans_mode_u8 == uint8(3)
                    rotateAngle_rad_f = single(pi / 2);
                else
                    rotateAngle_rad_f = single(-pi / 2);
                end

                [plan_result, num_u8] = changeRotationTransHandle( ...
                    p1, p2, rthSetRoll_deg_f, rotateAngle_rad_f, maxCircleNum_u8, ...
                    routePoint_struct, trans_mode_u8, cruiseSpd_mps_f, G_mps2_f, curHorizontalVelNEU_mps_f);

            end

            %% 圆弧规划成功

            if num_u8 > uint8(0) && num_u8 <= maxCircleNum_u8

                num_d = double(num_u8);

                %% 当前状态锚点

                returnRoute_struct(1).wpType_u8 = firstWpType_u8;
                returnRoute_struct(1).lat_deg_d = curLat_deg_d;
                returnRoute_struct(1).lon_deg_d = curLon_deg_d;
                returnRoute_struct(1).alt_m_f = curAlt_m_f;
                returnRoute_struct(1).yaw_deg_f = curYaw_deg_f;
                returnRoute_struct(1).spd_mps_f = horizontalSpd_mps_f;

                %% 固定翼规划航路

                returnRoute_struct(2:(num_d + 1)) = plan_result(1:num_d);

                % 第2点是首个实际执行的FW点。
                % 若沿用当前较低速度，可能无法满足前倾转换速度要求。
                if returnRoute_struct(2).wpType_u8 == WP_TYPE.FW_LINE_WP
                    returnRoute_struct(2).spd_mps_f = cruiseSpd_mps_f;
                end

                %% Home上方降落点

                landIdx_u8 = num_u8 + uint8(2);

                returnRoute_struct(landIdx_u8).wpType_u8 = WP_TYPE.MC_LAND_WP;
                returnRoute_struct(landIdx_u8).lat_deg_d = rthPointLat_deg_d;
                returnRoute_struct(landIdx_u8).lon_deg_d = rthPointLon_deg_d;
                returnRoute_struct(landIdx_u8).alt_m_f = fwRthAlt_m_f;
                returnRoute_struct(landIdx_u8).yaw_deg_f = rthEntryDirect_deg_f;
                returnRoute_struct(landIdx_u8).spd_mps_f = single(0.0);
                returnRoute_struct(landIdx_u8).yawType_u8 = uint8(2);

                %% 地面点

                groundIdx_u8 = landIdx_u8 + uint8(1);

                returnRoute_struct(groundIdx_u8) = returnRoute_struct(landIdx_u8);
                returnRoute_struct(groundIdx_u8).alt_m_f = rthGroundAlt_m_f;

                validNum_u8 = groundIdx_u8;
                routePlanValid_b = true;

            end

        end

    otherwise

        validNum_u8 = uint8(0);
        routePlanValid_b = false;

end

end