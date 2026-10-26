%% 同旋向水平过渡航线
function [point_struct, num_u8] = sameRotationTransHandle(p1, p2, rthSetRoll_deg_f, rotateAngle_rad_f, maxNum_u8, routePoint_struct, cruiseSpd_mps_f, G_mps2_f, curSpdNED_mps_f)

point_struct = repmat(routePoint_struct, maxNum_u8, 1);

% 计算起点圆弧圆心C1
rthSetRoll_deg_f = max(rthSetRoll_deg_f, single(0.1));
turn_radius = cruiseSpd_mps_f * cruiseSpd_mps_f / (G_mps2_f * tan(deg2rad(rthSetRoll_deg_f)));
v_a = [cos(deg2rad(p1.yaw_deg_f)), sin(deg2rad(p1.yaw_deg_f))];  % 这里横轴为n，纵轴为e

% 以p1为原点，建立NE坐标系，计算第一个圆心经纬度，注意n-e坐标系下旋转角度
v_1 = [cos(rotateAngle_rad_f), -sin(rotateAngle_rad_f); ...
       sin(rotateAngle_rad_f),  cos(rotateAngle_rad_f)] * v_a';
bearing_p1_to_C1 = rad2deg(atan2(v_1(2), v_1(1)));  % 计算与n轴的夹角，现在n轴为横轴
loc_refer_C1 = [p1.lat_deg_d, p1.lon_deg_d];
c_a_LL = offsetBearing(bearing_p1_to_C1, turn_radius, loc_refer_C1);  % 第一个圆心的经纬度

% 以p2为原点，建立NE坐标系，计算第二个圆心经纬度，注意n-e坐标系下旋转角度
v_b = [cos(deg2rad(p2.yaw_deg_f)), sin(deg2rad(p2.yaw_deg_f))];
v_2 = [cos(rotateAngle_rad_f), -sin(rotateAngle_rad_f); ...
       sin(rotateAngle_rad_f),  cos(rotateAngle_rad_f)] * v_b';
bearing_p2_to_C2 = rad2deg(atan2(v_2(2), v_2(1)));
loc_refer_C2 = [p2.lat_deg_d, p2.lon_deg_d];
c_b_LL = offsetBearing(bearing_p2_to_C2, turn_radius, loc_refer_C2);  % 第二个圆心的经纬度

e_norm = getDistanceNE(c_a_LL(1:2), c_b_LL(1:2));
v_C1_a1 = [cos(-rotateAngle_rad_f), -sin(-rotateAngle_rad_f); ...
           sin(-rotateAngle_rad_f),  cos(-rotateAngle_rad_f)] * e_norm';
bearing_C1_a1 = rad2deg(atan2(v_C1_a1(2), v_C1_a1(1)));
p_a1_LL = offsetBearing(bearing_C1_a1, turn_radius, c_a_LL(1:2));  % 第一个圆切出点的经纬度

v_C2_b0 = [cos(-rotateAngle_rad_f), -sin(-rotateAngle_rad_f); ...
           sin(-rotateAngle_rad_f),  cos(-rotateAngle_rad_f)] * e_norm';
bearing_C2_b0 = rad2deg(atan2(v_C2_b0(2), v_C2_b0(1)));
p_b0_LL = offsetBearing(bearing_C2_b0, turn_radius, c_b_LL(1:2));

% 构建航线
% 第一个航点
point_struct(1).wpType_u8 = WP_TYPE.FW_LINE_WP;  % 固定翼直线航点
point_struct(1).lat_deg_d = p1.lat_deg_d;
point_struct(1).lon_deg_d = p1.lon_deg_d;
point_struct(1).alt_m_f = p1.alt_m_f;
point_struct(1).yaw_deg_f = p1.yaw_deg_f;
point_struct(1).spd_mps_f = norm(curSpdNED_mps_f);  % 当前速度

% 第二个航点
point_struct(2).wpType_u8 = WP_TYPE.FW_CIR_WP;  % 圆弧航点
point_struct(2).lat_deg_d = c_a_LL(1);
point_struct(2).lon_deg_d = c_a_LL(2);
point_struct(2).alt_m_f = p1.alt_m_f;
if sign(rotateAngle_rad_f) > single(0)  % 逆时针是L；n-e坐标系逆时针转动，在e-n坐标系为顺时针转动
    point_struct(2).radius_m_f = turn_radius;
else
    point_struct(2).radius_m_f = -turn_radius;
end
point_struct(2).spd_mps_f = cruiseSpd_mps_f;

% 第三个航点
point_struct(3).wpType_u8 = WP_TYPE.FW_LINE_WP;  % 固定翼直线航点
point_struct(3).lat_deg_d = p_a1_LL(1);
point_struct(3).lon_deg_d = p_a1_LL(2);
point_struct(3).alt_m_f = p1.alt_m_f;
point_struct(3).spd_mps_f = cruiseSpd_mps_f;

% 第四个航点
point_struct(4).wpType_u8 = WP_TYPE.FW_LINE_WP;  % 固定翼直线航点
point_struct(4).lat_deg_d = p_b0_LL(1);
point_struct(4).lon_deg_d = p_b0_LL(2);
point_struct(4).alt_m_f = p2.alt_m_f;
point_struct(4).spd_mps_f = cruiseSpd_mps_f;

% 第五个航点
point_struct(5).wpType_u8 = WP_TYPE.FW_CIR_WP;  % 圆弧航点
point_struct(5).lat_deg_d = c_b_LL(1);
point_struct(5).lon_deg_d = c_b_LL(2);
point_struct(5).alt_m_f = p2.alt_m_f;
if sign(rotateAngle_rad_f) > single(0)  % 在n-e坐标系下逆时针转动，在e-n坐标系下是顺时针转动
    point_struct(5).radius_m_f = turn_radius;
else
    point_struct(5).radius_m_f = -turn_radius;
end
point_struct(5).spd_mps_f = cruiseSpd_mps_f;

% 第六个航点
point_struct(6).wpType_u8 = WP_TYPE.FW_LINE_WP;  % 固定翼直线航点
point_struct(6).lat_deg_d = p2.lat_deg_d;
point_struct(6).lon_deg_d = p2.lon_deg_d;
point_struct(6).alt_m_f = p2.alt_m_f;
point_struct(6).yaw_deg_f = p2.yaw_deg_f;
point_struct(6).spd_mps_f = cruiseSpd_mps_f;

% 有效航点数量
num_u8 = uint8(6);

end
