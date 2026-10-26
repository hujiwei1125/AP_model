function maxSpeed_mps_f = compute_max_speed_from_distance(jerkLimit_mps3_f, accelLimit_mps2_f, brakingDistance_m_f, ...
    finalSpeed_mps_f, smallValue_f)
% 根据剩余距离估算允许的最大当前速度。

if jerkLimit_mps3_f <= smallValue_f || accelLimit_mps2_f <= smallValue_f
    maxSpeed_mps_f = max(finalSpeed_mps_f, single(0.0));
    return;
end

coefficientB_mps_f = single(4.0) * accelLimit_mps2_f * accelLimit_mps2_f / jerkLimit_mps3_f;
coefficientC_m2ps2_f = single(-2.0) * accelLimit_mps2_f * max(brakingDistance_m_f, single(0.0)) - finalSpeed_mps_f * finalSpeed_mps_f;
discriminant_m2ps2_f = max(coefficientB_mps_f * coefficientB_mps_f - single(4.0) * coefficientC_m2ps2_f, single(0.0));
maxSpeed_mps_f = single(0.5) * (-coefficientB_mps_f + sqrt(discriminant_m2ps2_f));
maxSpeed_mps_f = max(maxSpeed_mps_f, finalSpeed_mps_f);
end
