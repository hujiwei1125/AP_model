function pos_deg_d = offset(ofsNorth_m_f, ofsEast_m_f, referenceLoc_struct)
%#codegen
%
% 功能：
% 按北向和东向距离偏移参考点水平位置
%
% 输入：
% ofsNorth_m_f        北向偏移距离，m，向北为正
% ofsEast_m_f         东向偏移距离，m，向东为正
% referenceLoc_struct 参考经纬度，[Lat, Lon]，deg
%
% 输出：
% pos_deg_d           偏移后的经纬度，[Lat, Lon]，deg

% 1 / (DEG_TO_RAD * RADIUS_OF_EARTH)
LOCATION_SCALING_FACTOR_INV = 8.983204953368922e-6;

%% 北向距离转换为纬度变化量
dLat_deg_d = double(ofsNorth_m_f) * LOCATION_SCALING_FACTOR_INV;

%% 经度方向缩放
longitudeScale_d = cos(deg2rad(referenceLoc_struct(1)));
longitudeScale_d = min(max(longitudeScale_d, 0.01), 1.0);

%% 东向距离转换为经度变化量
dLon_deg_d = double(ofsEast_m_f) * LOCATION_SCALING_FACTOR_INV / longitudeScale_d;

%% 计算新的经纬度
lat_deg_d = referenceLoc_struct(1) + dLat_deg_d;
lon_deg_d = referenceLoc_struct(2) + dLon_deg_d;

%% 经度限制到 [-180, 180)
% lon_deg_d = cldWrap180Double(lon_deg_d);

pos_deg_d = [lat_deg_d, lon_deg_d];

end