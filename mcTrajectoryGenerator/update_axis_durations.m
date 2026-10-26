function [velocitySetpoint_mps_f, direction_f, T1_s_f, T2_s_f, T3_s_f, accelerationInit_mps2_f, velocityInit_mps_f, localTime_s_f] = update_axis_durations(acceleration_mps2_f, velocity_mps_f, rawVelocitySetpoint_mps_f, maxVelocity_mps_f, maxAcceleration_mps2_f, maxJerk_mps3_f, smallValue_f)
% 根据当前一维状态和新的速度目标，重新规划下一周期使用的S曲线。
% 每次重规划都把localTime置0，后续由update_trajectory_axis从新的初始状态推进。

velocitySetpoint_mps_f = constrain_scalar(rawVelocitySetpoint_mps_f, -maxVelocity_mps_f, maxVelocity_mps_f);
localTime_s_f = single(0.0);
accelerationInit_mps2_f = acceleration_mps2_f;
velocityInit_mps_f = velocity_mps_f;

% 先估计如果只用最大jerk把当前加速度消除为0，届时速度会是多少。
velocityAtZeroAcceleration_mps_f = velocity_mps_f;
if abs(acceleration_mps2_f) > smallValue_f && maxJerk_mps3_f > smallValue_f
    jerkToZeroAcceleration_mps3_f = -sign(acceleration_mps2_f) * maxJerk_mps3_f;
    timeToZeroAcceleration_s_f = -acceleration_mps2_f / jerkToZeroAcceleration_mps3_f;
    velocityAtZeroAcceleration_mps_f = velocity_mps_f + acceleration_mps2_f * timeToZeroAcceleration_s_f + single(0.5) * jerkToZeroAcceleration_mps3_f * timeToZeroAcceleration_s_f * timeToZeroAcceleration_s_f;
end

% 规划方向由目标速度相对“零加速度时速度”的方向决定。
direction_f = sign(velocitySetpoint_mps_f - velocityAtZeroAcceleration_mps_f);
if direction_f == single(0.0)
    direction_f = sign(acceleration_mps2_f);
end

jerkStage1_mps3_f = direction_f * maxJerk_mps3_f;
deltaVelocity_mps_f = velocitySetpoint_mps_f - velocity_mps_f;

if abs(jerkStage1_mps3_f) > smallValue_f
    T1_s_f = compute_T1_minimum_time(acceleration_mps2_f, deltaVelocity_mps_f, jerkStage1_mps3_f, maxAcceleration_mps2_f, smallValue_f);
    T3_s_f = compute_T3(T1_s_f, acceleration_mps2_f, jerkStage1_mps3_f);
    T2_s_f = compute_T2_from_velocity(T1_s_f, T3_s_f, acceleration_mps2_f, deltaVelocity_mps_f, jerkStage1_mps3_f, smallValue_f);
else
    T1_s_f = single(0.0);
    T2_s_f = single(0.0);
    T3_s_f = single(0.0);
end
end
