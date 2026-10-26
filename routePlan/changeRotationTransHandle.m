function [point_struct, num_u8] = changeRotationTransHandle(p1, p2, rthSetRoll_deg_f, rotateAngle_rad_f, maxNum_u8, routePoint_struct, transMode_u8, ...
    cruiseSpd_mps_f, G_mps2_f, curSpdNED_mps_f)
%#codegen
% 变旋向水平过渡航线
%
% transMode_u8：
% 1：LSL；2：RSR；3：LSR；4：RSL。

point_struct = repmat(routePoint_struct, maxNum_u8, 1);

%% 计算起点圆弧圆心
rthSetRoll_deg_f = max(rthSetRoll_deg_f, single(0.1));
turn_radius = cruiseSpd_mps_f^2 / (G_mps2_f * tan(deg2rad(rthSetRoll_deg_f)));

v_a = [cos(deg2rad(p1.yaw_deg_f)), sin(deg2rad(p1.yaw_deg_f))];
v_1 = [cos(rotateAngle_rad_f), -sin(rotateAngle_rad_f); ...
       sin(rotateAngle_rad_f),  cos(rotateAngle_rad_f)] * v_a';
bearing_p1_to_C1 = rad2deg(atan2(v_1(2), v_1(1)));
loc_refer_C1 = [p1.lat_deg_d, p1.lon_deg_d];
c_a_LL = offsetBearing(bearing_p1_to_C1, turn_radius, loc_refer_C1);

%% 计算终点圆弧圆心
v_b = [cos(deg2rad(p2.yaw_deg_f)), sin(deg2rad(p2.yaw_deg_f))];
v_2 = [ cos(rotateAngle_rad_f), sin(rotateAngle_rad_f); ...
       -sin(rotateAngle_rad_f), cos(rotateAngle_rad_f)] * v_b';
bearing_p2_to_C2 = rad2deg(atan2(v_2(2), v_2(1)));
loc_refer_C2 = [p2.lat_deg_d, p2.lon_deg_d];
c_b_LL = offsetBearing(bearing_p2_to_C2, turn_radius, loc_refer_C2);

%% 计算两圆的内公切线切点
v_c1_c2 = getDistanceNE(c_a_LL(1:2), c_b_LL(1:2));
d = norm(v_c1_c2);

if d < 2 * turn_radius
    [point_struct, num_u8] = sameRotationTransHandle(p1, p2, rthSetRoll_deg_f, rotateAngle_rad_f, maxNum_u8, routePoint_struct, cruiseSpd_mps_f, G_mps2_f, curSpdNED_mps_f);
    return;
end

e_norm = v_c1_c2 / norm(v_c1_c2);
ang = acos(2 * turn_radius / d);

% transMode_u8：1(LSL)、2(RSR)、3(LSR)、4(RSL)
if transMode_u8 == 3
    ang = -ang;
end

v_C1_a1 = [cos(ang), -sin(ang); sin(ang), cos(ang)] * e_norm';
bearing_C1_a1 = rad2deg(atan2(v_C1_a1(2), v_C1_a1(1)));
p_a1_LL = offsetBearing(bearing_C1_a1, turn_radius, c_a_LL(1:2));

v_C2_b0 = [cos(pi + ang), -sin(pi + ang); ...
           sin(pi + ang),  cos(pi + ang)] * e_norm';
bearing_C2_b0 = rad2deg(atan2(v_C2_b0(2), v_C2_b0(1)));
p_b0_LL = offsetBearing(bearing_C2_b0, turn_radius, c_b_LL(1:2));

%% 构建航线
% 第一个航点：起点
point_struct(1).wpType_u8 = WP_TYPE.FW_LINE_WP;
point_struct(1).lat_deg_d = p1.lat_deg_d;
point_struct(1).lon_deg_d = p1.lon_deg_d;
point_struct(1).alt_m_f = p1.alt_m_f;
point_struct(1).yaw_deg_f = p1.yaw_deg_f;
point_struct(1).spd_mps_f = norm(curSpdNED_mps_f);

% 第二个航点：起点圆弧圆心
point_struct(2).wpType_u8 = WP_TYPE.FW_CIR_WP;
point_struct(2).lat_deg_d = c_a_LL(1);
point_struct(2).lon_deg_d = c_a_LL(2);
point_struct(2).alt_m_f = p1.alt_m_f;
if sign(rotateAngle_rad_f) == 1
    point_struct(2).radius_m_f = turn_radius;
else
    point_struct(2).radius_m_f = -turn_radius;
end
point_struct(2).spd_mps_f = cruiseSpd_mps_f;

% 第三个航点：第一段圆弧的切出点
point_struct(3).wpType_u8 = WP_TYPE.FW_LINE_WP;
point_struct(3).lat_deg_d = p_a1_LL(1);
point_struct(3).lon_deg_d = p_a1_LL(2);
point_struct(3).alt_m_f = p1.alt_m_f;
point_struct(3).spd_mps_f = cruiseSpd_mps_f;

% 第四个航点：第二段圆弧的切入点
point_struct(4).wpType_u8 = WP_TYPE.FW_LINE_WP;
point_struct(4).lat_deg_d = p_b0_LL(1);
point_struct(4).lon_deg_d = p_b0_LL(2);
point_struct(4).alt_m_f = p2.alt_m_f;
point_struct(4).spd_mps_f = cruiseSpd_mps_f;

% 第五个航点：终点圆弧圆心
point_struct(5).wpType_u8 = WP_TYPE.FW_CIR_WP;
point_struct(5).lat_deg_d = c_b_LL(1);
point_struct(5).lon_deg_d = c_b_LL(2);
point_struct(5).alt_m_f = p2.alt_m_f;
if sign(rotateAngle_rad_f) == -1
    point_struct(5).radius_m_f = turn_radius;
else
    point_struct(5).radius_m_f = -turn_radius;
end
point_struct(5).spd_mps_f = cruiseSpd_mps_f;

% 第六个航点：终点
point_struct(6).wpType_u8 = WP_TYPE.FW_LINE_WP;
point_struct(6).lat_deg_d = p2.lat_deg_d;
point_struct(6).lon_deg_d = p2.lon_deg_d;
point_struct(6).alt_m_f = p2.alt_m_f;
point_struct(6).yaw_deg_f = p2.yaw_deg_f;
point_struct(6).spd_mps_f = cruiseSpd_mps_f;

num_u8 = uint8(6);
end
