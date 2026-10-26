% 简化六自由度运动学模型初始化

simple6DofDt_s_f = single(0.05);

simple6DofRefLat_deg_d = 31.200000;
simple6DofRefLon_deg_d = 121.500000;
simple6DofRefAlt_m_f = single(100.0);
simple6DofGroundAlt_m_f = single(100.0);

% [N;E;U;Vn;Ve;Vu;roll;pitch;yaw;p;q;r]
simple6DofInitState_f = single(zeros(12, 1));
