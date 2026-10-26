function pos1_deg_d = offsetBearing(bearing_deg_f, distance_m_f, referenceLoc_struct)
%#codegen
%
% 功能：
% 根据方位角和距离偏移参考点水平位置
%
% 输入：
% bearing_deg_f       方位角，deg
%                     0 deg   = 北
%                     90 deg  = 东
%                     180 deg = 南
%                     270 deg = 西
%
% distance_m_f        偏移距离，m
% referenceLoc_struct 参考经纬度，[Lat, Lon]，deg
%
% 输出：
% pos1_deg_d          偏移后的经纬度，[Lat, Lon]，deg

%% 方位角转换为弧度
bearing_rad_f = deg2rad(bearing_deg_f);

%% 将距离分解到北、东方向
ofsNorth_m_f = distance_m_f * cos(bearing_rad_f);
ofsEast_m_f = distance_m_f * sin(bearing_rad_f);

%% 根据NE偏移计算新的经纬度
pos1_deg_d = offset(ofsNorth_m_f, ofsEast_m_f, referenceLoc_struct);

end