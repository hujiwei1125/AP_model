 %% 在局部NE坐标系中绘制实际飞行轨迹和航点航线
%
% 功能：
% 1. 从out.nav_struct提取经纬度和仿真时间；
% 2. 将经纬度转换到局部NE坐标系；
% 3. 绘制实际飞行轨迹和任务航点航线；
% 4. 点击实际轨迹时显示East、North和Time；
% 5. 将转换结果保存到navTrackLocal_struct。
%
% 坐标定义：
% 横轴：East，向东为正，m
% 纵轴：North，向北为正，m

%% 用户配置

navLogData = out.nav_struct;
routePlanData = missionRoutePlan_struct;

% true：首个有效航点作为原点
% false：首个有效导航点作为原点
useFirstWpAsOrigin_b = true;

% 第1行通常是仿真初始化数据，因此从第2行开始
navStartIdx_u32 = uint32(2);

% 绘图降采样倍数，1表示不降采样
plotDecimation_u32 = uint32(1);


%% 查找公共函数

scriptDir_c = fileparts(mfilename('fullpath'));

commonFunctionCandidate_c = {
    fullfile(scriptDir_c, 'commonFunction')
    fullfile(pwd, 'commonFunction')
    fullfile(pwd, 'AP_v10', 'commonFunction')
    };

if exist('getDistanceNE', 'file') ~= 2

    for candidateIdx_u32 = uint32(1):uint32(numel(commonFunctionCandidate_c))

        candidateDir_c = commonFunctionCandidate_c{candidateIdx_u32};

        if isfolder(candidateDir_c)
            addpath(candidateDir_c);
        end

    end

end

if exist('getDistanceNE', 'file') ~= 2
    error('找不到公共函数getDistanceNE.m，请检查commonFunction路径。');
end


%% 提取导航经纬度和时间

[navLat_deg_d, navLon_deg_d, navTime_s_d] = extractNavLatLonTime(navLogData);

% 如果nav_struct自身没有时间，则尝试使用out.tout
if isempty(navTime_s_d)
    navTime_s_d = extractSimulationOutputTime(out);
end

if isempty(navTime_s_d)
    error('无法获得nav_struct对应的仿真时间，请检查nav_struct时间信息或out.tout。');
end


%% 提取航点

[wpLat_deg_d, wpLon_deg_d, wpType_u8] = extractRouteLatLon(routePlanData);


%% 检查导航数据长度

if numel(navLat_deg_d) ~= numel(navLon_deg_d)
    error('导航记录中的纬度与经度数据长度不一致。');
end

if numel(navLat_deg_d) ~= numel(navTime_s_d)
    error('导航记录时间长度与经纬度长度不一致：位置=%u，时间=%u。', uint32(numel(navLat_deg_d)), uint32(numel(navTime_s_d)));
end


%% 从指定采样点开始取数据

if navStartIdx_u32 < uint32(1) || navStartIdx_u32 > uint32(numel(navLat_deg_d))
    error('navStartIdx_u32超出导航数据范围。');
end

navDataIdx_u32 = navStartIdx_u32:uint32(numel(navLat_deg_d));

navLat_deg_d = navLat_deg_d(navDataIdx_u32);
navLon_deg_d = navLon_deg_d(navDataIdx_u32);
navTime_s_d = navTime_s_d(navDataIdx_u32);


%% 去除无效导航数据

navValid_b = isfinite(navLat_deg_d) & isfinite(navLon_deg_d) & isfinite(navTime_s_d);
navValid_b = navValid_b & abs(navLat_deg_d) <= 90.0 & abs(navLon_deg_d) <= 180.0;

navLat_deg_d = navLat_deg_d(navValid_b);
navLon_deg_d = navLon_deg_d(navValid_b);
navTime_s_d = navTime_s_d(navValid_b);

if isempty(navLat_deg_d)
    error('导航记录中没有有效的经纬度数据。');
end


%% 去除无效航点

wpValid_b = isfinite(wpLat_deg_d) & isfinite(wpLon_deg_d);
wpValid_b = wpValid_b & abs(wpLat_deg_d) <= 90.0 & abs(wpLon_deg_d) <= 180.0;

wpLat_deg_d = wpLat_deg_d(wpValid_b);
wpLon_deg_d = wpLon_deg_d(wpValid_b);
wpType_u8 = wpType_u8(wpValid_b);

if isempty(wpLat_deg_d)
    error('航线中没有有效航点。');
end


%% 选择局部坐标系原点

if useFirstWpAsOrigin_b

    originLL_deg_d = [wpLat_deg_d(1), wpLon_deg_d(1)];
    originSource_c = '首个有效航点';

else

    originLL_deg_d = [navLat_deg_d(1), navLon_deg_d(1)];
    originSource_c = '首个有效导航点';

end


%% 导航经纬度转换为局部NE坐标

navNum_u32 = uint32(numel(navLat_deg_d));
navPosNE_m_f = single(zeros(double(navNum_u32), 2));

for navIdx_u32 = uint32(1):navNum_u32

    navLL_deg_d = [navLat_deg_d(navIdx_u32), navLon_deg_d(navIdx_u32)];
    navPosNE_m_f(navIdx_u32, :) = getDistanceNE(originLL_deg_d, navLL_deg_d);

end


%% 航点经纬度转换为局部NE坐标

wpNum_u32 = uint32(numel(wpLat_deg_d));
wpPosNE_m_f = single(zeros(double(wpNum_u32), 2));

for wpIdx_u32 = uint32(1):wpNum_u32

    wpLL_deg_d = [wpLat_deg_d(wpIdx_u32), wpLon_deg_d(wpIdx_u32)];
    wpPosNE_m_f(wpIdx_u32, :) = getDistanceNE(originLL_deg_d, wpLL_deg_d);

end


%% 绘图降采样

plotStep_u32 = max(plotDecimation_u32, uint32(1));
plotIdx_u32 = uint32(1):plotStep_u32:navNum_u32;

if plotIdx_u32(end) ~= navNum_u32
    plotIdx_u32(end + 1) = navNum_u32;
end

navPlotNE_m_f = navPosNE_m_f(plotIdx_u32, :);
navPlotTime_s_d = navTime_s_d(plotIdx_u32);


%% 绘制轨迹

figure('Name', '局部NE航迹与航点航线', 'Color', 'w');

hold on;
grid on;
box on;
axis equal;


%% 实际飞行轨迹

actualTrajHandle = plot(navPlotNE_m_f(:, 2), navPlotNE_m_f(:, 1), 'b-', 'LineWidth', 1.6, 'DisplayName', '实际飞行轨迹');


%% 实际轨迹Data Tip

actualTrajHandle.DataTipTemplate.DataTipRows(1).Label = 'East / m';
actualTrajHandle.DataTipTemplate.DataTipRows(2).Label = 'North / m';

actualTrajHandle.DataTipTemplate.DataTipRows(1).Format = '%.4f';
actualTrajHandle.DataTipTemplate.DataTipRows(2).Format = '%.4f';

timeDataTipRow = dataTipTextRow('Time / s', navPlotTime_s_d);
timeDataTipRow.Format = '%.3f';

actualTrajHandle.DataTipTemplate.DataTipRows(end + 1) = timeDataTipRow;


%% 航点航线

wpRouteHandle = plot(wpPosNE_m_f(:, 2), wpPosNE_m_f(:, 1), 'r--o', 'LineWidth', 1.4, 'MarkerSize', 7, 'MarkerFaceColor', 'w', 'DisplayName', '航点组成的航线');


%% 实际轨迹起点

startHandle = plot(navPosNE_m_f(1, 2), navPosNE_m_f(1, 1), 'gs', 'MarkerSize', 9, 'MarkerFaceColor', 'g', 'DisplayName', '实际轨迹起点');


%% 实际轨迹终点

endHandle = plot(navPosNE_m_f(end, 2), navPosNE_m_f(end, 1), 'md', 'MarkerSize', 9, 'MarkerFaceColor', 'm', 'DisplayName', '实际轨迹终点');


%% 标注航点

for wpIdx_u32 = uint32(1):wpNum_u32

    wpLabel_c = sprintf('  WP%u  type=%u', wpIdx_u32, wpType_u8(wpIdx_u32));

    text(wpPosNE_m_f(wpIdx_u32, 2), wpPosNE_m_f(wpIdx_u32, 1), wpLabel_c, 'Color', [0.75, 0.0, 0.0], 'FontSize', 9, 'VerticalAlignment', 'bottom');

end


%% 坐标轴

xlabel('East / m');
ylabel('North / m');

title(sprintf('局部NE航迹图（原点：%s，Lat=%.8f°，Lon=%.8f°）', originSource_c, originLL_deg_d(1), originLL_deg_d(2)));

legend([actualTrajHandle, wpRouteHandle, startHandle, endHandle], 'Location', 'best');

hold off;


%% 保存转换结果

navTrackLocal_struct = struct();

navTrackLocal_struct.originLL_deg_d = originLL_deg_d;

navTrackLocal_struct.navTime_s_d = navTime_s_d;

navTrackLocal_struct.navLat_deg_d = navLat_deg_d;
navTrackLocal_struct.navLon_deg_d = navLon_deg_d;

navTrackLocal_struct.navPosN_m_f = navPosNE_m_f(:, 1);
navTrackLocal_struct.navPosE_m_f = navPosNE_m_f(:, 2);

navTrackLocal_struct.wpLat_deg_d = wpLat_deg_d;
navTrackLocal_struct.wpLon_deg_d = wpLon_deg_d;

navTrackLocal_struct.wpPosN_m_f = wpPosNE_m_f(:, 1);
navTrackLocal_struct.wpPosE_m_f = wpPosNE_m_f(:, 2);

navTrackLocal_struct.wpType_u8 = wpType_u8;

disp('局部NE航迹绘制完成，转换结果已保存到navTrackLocal_struct。');


%% ========================================================================
% 从导航记录中提取经纬度和时间
% =========================================================================

function [lat_deg_d, lon_deg_d, time_s_d] = extractNavLatLonTime(navData)

time_s_d = [];


%% Simulink.SimulationData.Signal

if isa(navData, 'Simulink.SimulationData.Signal')

    [lat_deg_d, lon_deg_d, time_s_d] = extractNavLatLonTime(navData.Values);
    return;

end


%% Dataset

if isa(navData, 'Simulink.SimulationData.Dataset')

    if numElements(navData) ~= 1
        error('navLogData为Dataset时应只包含一个nav_struct信号。');
    end

    signalData = navData.getElement(1);
    [lat_deg_d, lon_deg_d, time_s_d] = extractNavLatLonTime(signalData);

    return;

end


%% timeseries

if isa(navData, 'timeseries')

    time_s_d = double(navData.Time(:));
    [lat_deg_d, lon_deg_d, ~] = extractNavLatLonTime(navData.Data);

    return;

end


%% timetable

if istimetable(navData)

    lat_deg_d = double(navData.lat_deg_d(:));
    lon_deg_d = double(navData.lon_deg_d(:));

    rowTime = navData.Properties.RowTimes;

    if isduration(rowTime)

        time_s_d = double(seconds(rowTime(:)));

    elseif isdatetime(rowTime)

        time_s_d = double(seconds(rowTime(:) - rowTime(1)));

    end

    return;

end


%% table

if istable(navData)

    lat_deg_d = double(navData.lat_deg_d(:));
    lon_deg_d = double(navData.lon_deg_d(:));

    variableNames_c = navData.Properties.VariableNames;

    if any(strcmp(variableNames_c, 'Time'))

        time_s_d = convertTimeToSeconds(navData.Time);

    elseif any(strcmp(variableNames_c, 'time'))

        time_s_d = convertTimeToSeconds(navData.time);

    elseif any(strcmp(variableNames_c, 'tout'))

        time_s_d = convertTimeToSeconds(navData.tout);

    end

    return;

end


%% struct

if isstruct(navData)

    if isscalar(navData) && isfield(navData, 'Values')

        [lat_deg_d, lon_deg_d, time_s_d] = extractNavLatLonTime(navData.Values);
        return;

    end


    if isscalar(navData) && isfield(navData, 'time') && isfield(navData, 'signals') && isfield(navData.signals, 'values')

        time_s_d = double(navData.time(:));
        [lat_deg_d, lon_deg_d, ~] = extractNavLatLonTime(navData.signals.values);

        return;

    end


    if isscalar(navData) && isfield(navData, 'signals') && isfield(navData.signals, 'values')

        [lat_deg_d, lon_deg_d, time_s_d] = extractNavLatLonTime(navData.signals.values);
        return;

    end


    if isfield(navData, 'lat_deg_d') && isfield(navData, 'lon_deg_d')

        if isscalar(navData)

            lat_deg_d = extractNumericSignal(navData.lat_deg_d);
            lon_deg_d = extractNumericSignal(navData.lon_deg_d);

            time_s_d = extractSignalTime(navData.lat_deg_d);

            if isempty(time_s_d)
                time_s_d = extractSignalTime(navData.lon_deg_d);
            end

            if isempty(time_s_d) && isfield(navData, 'time')
                time_s_d = double(navData.time(:));
            end

            if isempty(time_s_d) && isfield(navData, 'Time')
                time_s_d = convertTimeToSeconds(navData.Time);
            end

        else

            lat_deg_d = double(reshape([navData.lat_deg_d], [], 1));
            lon_deg_d = double(reshape([navData.lon_deg_d], [], 1));

            if isfield(navData, 'time')
                time_s_d = double(reshape([navData.time], [], 1));
            elseif isfield(navData, 'Time')
                time_s_d = double(reshape([navData.Time], [], 1));
            end

        end

        return;

    end

end


%% 数值矩阵

if isnumeric(navData) && ismatrix(navData) && size(navData, 2) >= 2

    lat_deg_d = double(navData(:, 1));
    lon_deg_d = double(navData(:, 2));

    return;

end

error('无法从navLogData中提取经纬度。');

end


%% ========================================================================
% 从SimulationOutput中提取时间
% =========================================================================

function time_s_d = extractSimulationOutputTime(simOutData)

time_s_d = [];

if isa(simOutData, 'Simulink.SimulationOutput')

    try
        toutData = simOutData.get('tout');
        time_s_d = extractTimeVector(toutData);
    catch
        time_s_d = [];
    end

    return;

end


if isstruct(simOutData)

    if isfield(simOutData, 'tout')
        time_s_d = extractTimeVector(simOutData.tout);
    end

    return;

end


try

    time_s_d = extractTimeVector(simOutData.tout);

catch

    time_s_d = [];

end

end


%% ========================================================================
% 提取时间向量
% =========================================================================

function time_s_d = extractTimeVector(timeData)

time_s_d = [];

if isnumeric(timeData)

    time_s_d = double(timeData(:));
    return;

end


if isduration(timeData)

    time_s_d = double(seconds(timeData(:)));
    return;

end


if isdatetime(timeData)

    time_s_d = double(seconds(timeData(:) - timeData(1)));
    return;

end


if isa(timeData, 'timeseries')

    if isnumeric(timeData.Data)
        time_s_d = double(timeData.Data(:));
    else
        time_s_d = double(timeData.Time(:));
    end

    return;

end


if isa(timeData, 'Simulink.SimulationData.Signal')

    time_s_d = extractTimeVector(timeData.Values);
    return;

end


if isstruct(timeData)

    if isfield(timeData, 'time')

        time_s_d = double(timeData.time(:));
        return;

    end

    if isfield(timeData, 'Time')

        time_s_d = convertTimeToSeconds(timeData.Time);
        return;

    end

    if isfield(timeData, 'signals') && isfield(timeData.signals, 'values')

        time_s_d = double(timeData.signals.values(:));
        return;

    end

end

end


%% ========================================================================
% 从单个信号中提取时间
% =========================================================================

function time_s_d = extractSignalTime(signalData)

time_s_d = [];

if isa(signalData, 'timeseries')

    time_s_d = double(signalData.Time(:));
    return;

end


if isa(signalData, 'Simulink.SimulationData.Signal')

    time_s_d = extractSignalTime(signalData.Values);
    return;

end


if istimetable(signalData)

    rowTime = signalData.Properties.RowTimes;

    if isduration(rowTime)
        time_s_d = double(seconds(rowTime(:)));
    elseif isdatetime(rowTime)
        time_s_d = double(seconds(rowTime(:) - rowTime(1)));
    end

    return;

end


if isstruct(signalData)

    if isfield(signalData, 'time')

        time_s_d = double(signalData.time(:));
        return;

    end

    if isfield(signalData, 'Time')

        time_s_d = convertTimeToSeconds(signalData.Time);
        return;

    end

    if isfield(signalData, 'Values')

        time_s_d = extractSignalTime(signalData.Values);
        return;

    end

end

end


%% ========================================================================
% 时间转换为秒
% =========================================================================

function time_s_d = convertTimeToSeconds(timeData)

if isnumeric(timeData)

    time_s_d = double(timeData(:));

elseif isduration(timeData)

    time_s_d = double(seconds(timeData(:)));

elseif isdatetime(timeData)

    time_s_d = double(seconds(timeData(:) - timeData(1)));

else

    time_s_d = [];

end

end


%% ========================================================================
% 从航线结构中提取有效航点
% =========================================================================

function [lat_deg_d, lon_deg_d, wpType_u8] = extractRouteLatLon(routePlan)

if ~isstruct(routePlan)
    error('routePlanData必须是航线结构体。');
end


if isscalar(routePlan) && isfield(routePlan, 'missionRoute_struct')

    routeStruct = routePlan.missionRoute_struct;
    validWpNum_u32 = uint32(routePlan.validWpNum_u8);

elseif isscalar(routePlan) && isfield(routePlan, 'route_struct')

    routeStruct = routePlan.route_struct;
    validWpNum_u32 = uint32(routePlan.validWpNum_u8);

elseif isfield(routePlan, 'lat_deg_d') && isfield(routePlan, 'lon_deg_d')

    routeStruct = routePlan;
    validWpNum_u32 = uint32(numel(routeStruct));

else

    error('无法从routePlanData中识别航点数组。');

end


validWpNum_u32 = min(validWpNum_u32, uint32(numel(routeStruct)));

if validWpNum_u32 < uint32(1)
    error('航线有效航点数量为0。');
end


routeStruct = routeStruct(1:validWpNum_u32);

lat_deg_d = double(reshape([routeStruct.lat_deg_d], [], 1));
lon_deg_d = double(reshape([routeStruct.lon_deg_d], [], 1));


if isfield(routeStruct, 'wpType_u8')

    wpType_u8 = uint8(reshape([routeStruct.wpType_u8], [], 1));

else

    wpType_u8 = uint8(zeros(double(validWpNum_u32), 1));

end

end


%% ========================================================================
% 提取数值信号
% =========================================================================

function value_d = extractNumericSignal(signalData)

if isa(signalData, 'timeseries')

    value_d = double(signalData.Data(:));

elseif isa(signalData, 'Simulink.SimulationData.Signal')

    value_d = extractNumericSignal(signalData.Values);

elseif isstruct(signalData) && isfield(signalData, 'signals') && isfield(signalData.signals, 'values')

    value_d = double(signalData.signals.values(:));

elseif isnumeric(signalData)

    value_d = double(signalData(:));

else

    error('经纬度字段不是支持的数值信号格式。');

end

end