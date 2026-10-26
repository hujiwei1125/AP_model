function [jerk_mps3_f, acceleration_mps2_f, velocity_mps_f, position_m_f] = evaluate_polynomial(jerkMagnitude_mps3_f, accelerationInitial_mps2_f, velocityInitial_mps_f, positionInitial_m_f, time_s_f, direction_f)
% 在一段恒定jerk区间内，用解析多项式直接计算状态。
% jerk = direction * jerkMagnitude
% a(t) = a0 + j*t
% v(t) = v0 + a0*t + 0.5*j*t^2
% x(t) = x0 + v0*t + 0.5*a0*t^2 + j*t^3/6

jerk_mps3_f = direction_f * jerkMagnitude_mps3_f;
timeSquared_s2_f = time_s_f * time_s_f;
timeCubed_s3_f = timeSquared_s2_f * time_s_f;
acceleration_mps2_f = accelerationInitial_mps2_f + jerk_mps3_f * time_s_f;
velocity_mps_f = velocityInitial_mps_f + accelerationInitial_mps2_f * time_s_f + 0.5 * jerk_mps3_f * timeSquared_s2_f;
position_m_f = positionInitial_m_f + velocityInitial_mps_f * time_s_f + 0.5 * accelerationInitial_mps2_f * timeSquared_s2_f + (1.0 / 6.0) * jerk_mps3_f * timeCubed_s3_f;
end
