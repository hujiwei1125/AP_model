function yaw_deg_f = calcHorizonYaw(dirN_m_f, dirE_m_f, fallbackYaw_deg_f)
%#codegen
%
% 根据水平NE方向矢量计算偏航角。
% 当水平矢量长度过小时返回退化偏航角。

%% 常量

minYawDirDistance_m_f = single(0.1);
rad2deg_f = single(57.29577951308232);

%% 计算水平方向矢量长度平方

directionLengthSq_m2_f = sqrt(dirN_m_f * dirN_m_f + dirE_m_f * dirE_m_f);

if directionLengthSq_m2_f > minYawDirDistance_m_f

    yaw_deg_f = single(atan2(dirE_m_f, dirN_m_f)) * rad2deg_f;
    yaw_deg_f = cldWrap180(yaw_deg_f);

else

    yaw_deg_f = cldWrap180(fallbackYaw_deg_f);

end

end