%% 设定home点位置
setPoint_struct = struct('lat_deg_d',double(31.200000),'lon_deg_d',double(121.500000),'alt_m_f',single(100),'yaw_deg_f',single(0));

%% 生成航线
maxNumWp = uint8(30);
wp_struct = struct('wpType_u8',WP_TYPE.INVALID_WP,'lat_deg_d',double(0),'lon_deg_d',double(0),...
    'alt_m_f',single(0),'spd_mps_f',single(0),'radius_m_f',single(0),'fastWp_b',false,'yawType_u8',uint8(0), ...
    'yaw_deg_f',single(0),'timeOrCount_u16',uint16(0),'arrivalCond_u8',uint8(0));
missionRoute_struct = repmat(wp_struct, maxNumWp, 1);
validWpNum_u8 = uint8(5);

missionRoutePlan_struct.validWpNum_u8 = validWpNum_u8;
missionRoutePlan_struct.missionRoute_struct = missionRoute_struct;

%% 旋翼航线
missionRoutePlan_struct.missionRoute_struct(1).wpType_u8 = WP_TYPE.MC_TAKEOFF_WP;
missionRoutePlan_struct.missionRoute_struct(1).lat_deg_d = double(31.200000);
missionRoutePlan_struct.missionRoute_struct(1).lon_deg_d = double(121.500000);
missionRoutePlan_struct.missionRoute_struct(1).alt_m_f = setPoint_struct.alt_m_f + single(30);
missionRoutePlan_struct.missionRoute_struct(1).fastWp_b = false;
missionRoutePlan_struct.missionRoute_struct(1).yaw_deg_f = single(0);
missionRoutePlan_struct.missionRoute_struct(1).yawType_u8 = uint8(1);

missionRoutePlan_struct.missionRoute_struct(2).wpType_u8 = WP_TYPE.MC_LINE_WP;
% missionRoutePlan_struct.missionRoute_struct(2).lat_deg_d = double(31.200000);
% missionRoutePlan_struct.missionRoute_struct(2).lon_deg_d = double(121.500090);
missionRoutePlan_struct.missionRoute_struct(2).lat_deg_d = double(31.200000);
missionRoutePlan_struct.missionRoute_struct(2).lon_deg_d = double(121.502000);
missionRoutePlan_struct.missionRoute_struct(2).alt_m_f = setPoint_struct.alt_m_f + single(30);
missionRoutePlan_struct.missionRoute_struct(2).fastWp_b = true;
missionRoutePlan_struct.missionRoute_struct(2).yawType_u8 = uint8(2);

missionRoutePlan_struct.missionRoute_struct(3).wpType_u8 = WP_TYPE.MC_LINE_WP;
missionRoutePlan_struct.missionRoute_struct(3).lat_deg_d = double(31.203000);
missionRoutePlan_struct.missionRoute_struct(3).lon_deg_d = double(121.502000);
missionRoutePlan_struct.missionRoute_struct(3).alt_m_f = setPoint_struct.alt_m_f + single(20);
missionRoutePlan_struct.missionRoute_struct(3).fastWp_b = true;
missionRoutePlan_struct.missionRoute_struct(3).yawType_u8 = uint8(2);

missionRoutePlan_struct.missionRoute_struct(4).wpType_u8 = WP_TYPE.MC_LINE_WP;
missionRoutePlan_struct.missionRoute_struct(4).lat_deg_d = double(31.205000);
missionRoutePlan_struct.missionRoute_struct(4).lon_deg_d = double(121.505000);
missionRoutePlan_struct.missionRoute_struct(4).alt_m_f = setPoint_struct.alt_m_f + single(40);
missionRoutePlan_struct.missionRoute_struct(4).fastWp_b = false;
missionRoutePlan_struct.missionRoute_struct(4).yawType_u8 = uint8(2);

missionRoutePlan_struct.missionRoute_struct(5).wpType_u8 = WP_TYPE.MC_LAND_WP;
% missionRoutePlan_struct.missionRoute_struct(3).lat_deg_d = double(31.200080);
% missionRoutePlan_struct.missionRoute_struct(3).lon_deg_d = double(121.500180);
missionRoutePlan_struct.missionRoute_struct(5).lat_deg_d = double(31.200000);
missionRoutePlan_struct.missionRoute_struct(5).lon_deg_d = double(121.500000);
missionRoutePlan_struct.missionRoute_struct(5).alt_m_f = setPoint_struct.alt_m_f + single(16);
missionRoutePlan_struct.missionRoute_struct(5).fastWp_b = false;
missionRoutePlan_struct.missionRoute_struct(5).yawType_u8 = uint8(0);

%% 固定翼
% missionRoutePlan_struct.missionRoute_struct(1).wpType_u8 = WP_TYPE.MC_TAKEOFF_WP;
% missionRoutePlan_struct.missionRoute_struct(1).lat_deg_d = double(31.200000);
% missionRoutePlan_struct.missionRoute_struct(1).lon_deg_d = double(121.500000);
% missionRoutePlan_struct.missionRoute_struct(1).alt_m_f = setPoint_struct.alt_m_f + single(40);
% missionRoutePlan_struct.missionRoute_struct(1).fastWp_b = false;
% missionRoutePlan_struct.missionRoute_struct(1).yaw_deg_f = single(0);
% missionRoutePlan_struct.missionRoute_struct(1).yawType_u8 = uint8(1);
% 
% missionRoutePlan_struct.missionRoute_struct(2).wpType_u8 = WP_TYPE.FW_LINE_WP;
% missionRoutePlan_struct.missionRoute_struct(2).lat_deg_d = double(31.200000);
% missionRoutePlan_struct.missionRoute_struct(2).lon_deg_d = double(121.505000);
% missionRoutePlan_struct.missionRoute_struct(2).alt_m_f = setPoint_struct.alt_m_f + single(40);
% missionRoutePlan_struct.missionRoute_struct(2).fastWp_b = true;
% missionRoutePlan_struct.missionRoute_struct(2).yawType_u8 = uint8(2);
% 
% missionRoutePlan_struct.missionRoute_struct(3).wpType_u8 = WP_TYPE.FW_LINE_WP;
% missionRoutePlan_struct.missionRoute_struct(3).lat_deg_d = double(31.209000);
% missionRoutePlan_struct.missionRoute_struct(3).lon_deg_d = double(121.518000);
% missionRoutePlan_struct.missionRoute_struct(3).alt_m_f = setPoint_struct.alt_m_f + single(40);
% missionRoutePlan_struct.missionRoute_struct(3).fastWp_b = true;
% missionRoutePlan_struct.missionRoute_struct(3).yawType_u8 = uint8(2);
% 
% missionRoutePlan_struct.missionRoute_struct(4).wpType_u8 = WP_TYPE.FW_LINE_WP;
% missionRoutePlan_struct.missionRoute_struct(4).lat_deg_d = double(31.205000);
% missionRoutePlan_struct.missionRoute_struct(4).lon_deg_d = double(121.520000);
% missionRoutePlan_struct.missionRoute_struct(4).alt_m_f = setPoint_struct.alt_m_f + single(40);
% missionRoutePlan_struct.missionRoute_struct(4).fastWp_b = false;
% missionRoutePlan_struct.missionRoute_struct(4).yawType_u8 = uint8(2);
% 
% missionRoutePlan_struct.missionRoute_struct(5).wpType_u8 = WP_TYPE.MC_LAND_WP;
% missionRoutePlan_struct.missionRoute_struct(5).lat_deg_d = double(31.20000);
% % missionRoutePlan_struct.missionRoute_struct(5).lon_deg_d = double(121.540000);
% missionRoutePlan_struct.missionRoute_struct(5).lon_deg_d = double(121.500000);
% missionRoutePlan_struct.missionRoute_struct(5).alt_m_f = setPoint_struct.alt_m_f + single(15);
% missionRoutePlan_struct.missionRoute_struct(5).fastWp_b = false;
% missionRoutePlan_struct.missionRoute_struct(5).yawType_u8 = uint8(0);


