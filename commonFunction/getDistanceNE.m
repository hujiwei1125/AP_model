function posNE_m_f = getDistanceNE(loc1_struct, loc2_struct)
%#codegen
%
% 功能：
% 根据两个经纬度坐标计算局部NE方向的位置差。
%
% 输入：
% loc1_struct = [Lat1, Lon1]，deg
% loc2_struct = [Lat2, Lon2]，deg
%
% 输出：
% posNE_m_f(1)：北向距离，m
% posNE_m_f(2)：东向距离，m
%
% 坐标系：
% N：北向为正
% E：东向为正

%% 经纬度距离比例系数
% DEG_TO_RAD * RADIUS_OF_EARTH
LOCATION_SCALING_FACTOR = 111318.84502145034;

%% 北向距离
posN_m_f = single((loc2_struct(1) - loc1_struct(1)) * LOCATION_SCALING_FACTOR);

%% 经度方向缩放系数
longitudeScale_f = min(max(cos(deg2rad(loc1_struct(1))), 0.01), 1.0);

%% 东向距离
deltaLon_deg_f = cldWrap180(loc2_struct(2) - loc1_struct(2));

posE_m_f = single(deltaLon_deg_f * LOCATION_SCALING_FACTOR * longitudeScale_f);

%% NE距离
posNE_m_f = [posN_m_f, posE_m_f];

end