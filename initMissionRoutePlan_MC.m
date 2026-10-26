%% V10 missionRoutePlan_struct：纯旋翼方形航线
%
% 使用方法：
% 1. 先在 AP_v10 目录运行 APInit，使 WP_TYPE 和 Bus 定义可用；
% 2. 再运行本脚本；
% 3. 将生成的 missionRoutePlan_struct 接入 GuidanceLaw 的同名输入。
%
% V10 外部航线约定：
% 1. 外部航线只包含实际任务点，不包含当前位置起飞锚点；
% 2. 第一个有效点必须是 MC_TAKEOFF_WP；
% 3. 最后一个有效点必须是 MC_LAND_WP；
% 4. tryRouteFlightPlan 会自动增加起飞锚点和垂直降落执行点。

%% 可修改的航线基准参数

% 与 V10/simple6dof_plant/initSimple6DofPlant.m 的默认初始位置一致。
takeoffLat_deg_d = 31.200000;
takeoffLon_deg_d = 121.500000;
takeoffGroundAlt_m_f = single(100.0);      % 起飞场地海拔高度
routeHeightAgl_m_f = single(20.0);         % 航线相对起飞场地高度
routeAlt_m_f = takeoffGroundAlt_m_f + routeHeightAgl_m_f;

% 每行是相对起飞点的 [北向偏移, 东向偏移]，单位 m。
% 航线：原地起飞 -> 向北 -> 向东 -> 向南 -> 向西返回起飞点降落。
routeOffsetNE_m_f = single([ ...
     0.0,  0.0; ...                        % WP1：垂直起飞目标
    80.0,  0.0; ...                        % WP2：旋翼直线点
    80.0, 80.0; ...                        % WP3：旋翼直线点
     0.0, 80.0; ...                        % WP4：旋翼直线点
     0.0,  0.0]);                          % WP5：垂直降落进入点

%% 检查 V10 枚举是否已经加载

try
    invalidWp_enum = WP_TYPE.INVALID_WP;
catch
    error('未加载 WP_TYPE。请先在 AP_v10 目录运行 APInit，再运行本脚本。');
end

%% 创建与 wayPointBus 完全一致的默认航点

maxWpNum_u8 = uint8(30);

wpDefault_struct = struct( ...
    'wpType_u8', invalidWp_enum, ...
    'lat_deg_d', double(0.0), ...
    'lon_deg_d', double(0.0), ...
    'alt_m_f', single(0.0), ...
    'spd_mps_f', single(0.0), ...
    'radius_m_f', single(0.0), ...
    'fastWp_b', false, ...
    'yawType_u8', uint8(0), ...
    'yaw_deg_f', single(0.0), ...
    'timeOrCount_u16', uint16(0), ...
    'arrivalCond_u8', uint8(0));

missionRoutePlan_struct = struct( ...
    'validWpNum_u8', uint8(5), ...
    'missionRoute_struct', repmat(wpDefault_struct, double(maxWpNum_u8), 1));

%% 计算各航点经纬度并填写公共字段

takeoffLoc_deg_d = [takeoffLat_deg_d, takeoffLon_deg_d];

for wpIdx_u8 = uint8(1):missionRoutePlan_struct.validWpNum_u8

    wpLoc_deg_d = offset( ...
        routeOffsetNE_m_f(wpIdx_u8, 1), ...
        routeOffsetNE_m_f(wpIdx_u8, 2), ...
        takeoffLoc_deg_d);

    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).lat_deg_d = double(wpLoc_deg_d(1));
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).lon_deg_d = double(wpLoc_deg_d(2));
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).alt_m_f = routeAlt_m_f;
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).radius_m_f = single(0.0);
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).yawType_u8 = uint8(0);
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).yaw_deg_f = single(0.0);
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).timeOrCount_u16 = uint16(0);
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).arrivalCond_u8 = uint8(0);

end

%% WP1：垂直起飞目标点

missionRoutePlan_struct.missionRoute_struct(1).wpType_u8 = WP_TYPE.MC_TAKEOFF_WP;
missionRoutePlan_struct.missionRoute_struct(1).spd_mps_f = single(0.0);
missionRoutePlan_struct.missionRoute_struct(1).fastWp_b = false;

%% WP2～WP4：旋翼直线快速过点

for wpIdx_u8 = uint8(2):uint8(4)

    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).wpType_u8 = WP_TYPE.MC_LINE_WP;
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).spd_mps_f = single(7.0);
    missionRoutePlan_struct.missionRoute_struct(wpIdx_u8).fastWp_b = true;

end

%% WP5：降落位置进入点

% 此点高度是进入垂直下降前的海拔高度，不是地面海拔高度。
% V10 会复制该点生成第二个 MC_LAND_WP；真正下降速度由 hag_m_f 导引产生。
missionRoutePlan_struct.missionRoute_struct(5).wpType_u8 = WP_TYPE.MC_LAND_WP;
missionRoutePlan_struct.missionRoute_struct(5).spd_mps_f = single(0.0);
missionRoutePlan_struct.missionRoute_struct(5).fastWp_b = false;

%% 基本一致性检查

assert(missionRoutePlan_struct.validWpNum_u8 >= uint8(2));
assert(missionRoutePlan_struct.validWpNum_u8 <= uint8(28));
assert(missionRoutePlan_struct.missionRoute_struct(1).wpType_u8 == WP_TYPE.MC_TAKEOFF_WP);
assert(missionRoutePlan_struct.missionRoute_struct(missionRoutePlan_struct.validWpNum_u8).wpType_u8 == WP_TYPE.MC_LAND_WP);

disp('已生成 V10 旋翼航线输入 missionRoutePlan_struct。');
disp('外部有效航点：5；模型内部规划后有效航点：7。');
