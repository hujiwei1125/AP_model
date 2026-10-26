function [jerk_mps3_f, acceleration_mps2_f, velocity_mps_f, position_m_f] = update_trajectory_axis(accelerationInit_mps2_f, velocityInit_mps_f, ...
    positionInit_m_f, T1_s_f, T2_s_f, T3_s_f, maxJerk_mps3_f, direction_f, dt_s_f, timeStretch_f)
% 按照“正向jerk -> 匀加速度 -> 反向jerk -> 匀速”的顺序推进一维轨迹。
% timeStretch_f=1表示正常推进；小于1表示减慢虚拟轨迹时间。

timeRemaining_s_f = dt_s_f * timeStretch_f;

% 第一阶段：恒定jerk改变加速度。
timeStage1_s_f = min(timeRemaining_s_f, T1_s_f);
[jerk_mps3_f, acceleration_mps2_f, velocity_mps_f, position_m_f] = evaluate_polynomial(maxJerk_mps3_f, accelerationInit_mps2_f, velocityInit_mps_f, positionInit_m_f, timeStage1_s_f, direction_f);
timeRemaining_s_f = timeRemaining_s_f - timeStage1_s_f;

% 第二阶段：jerk为0，保持第一阶段末端加速度。
if timeRemaining_s_f > single(0.0)
    timeStage2_s_f = min(timeRemaining_s_f, T2_s_f);
    [jerk_mps3_f, acceleration_mps2_f, velocity_mps_f, position_m_f] = evaluate_polynomial(single(0.0), acceleration_mps2_f, velocity_mps_f, position_m_f, timeStage2_s_f, single(0.0));
    timeRemaining_s_f = timeRemaining_s_f - timeStage2_s_f;
end

% 第三阶段：使用反向jerk把加速度平滑拉回0。
if timeRemaining_s_f > single(0.0)
    timeStage3_s_f = min(timeRemaining_s_f, T3_s_f);
    [jerk_mps3_f, acceleration_mps2_f, velocity_mps_f, position_m_f] = evaluate_polynomial(maxJerk_mps3_f, acceleration_mps2_f, velocity_mps_f, position_m_f, timeStage3_s_f, -direction_f);
    timeRemaining_s_f = timeRemaining_s_f - timeStage3_s_f;
end

% 三个阶段结束后，保持当前速度继续积分位置。
if timeRemaining_s_f > single(0.0)
    [jerk_mps3_f, acceleration_mps2_f, velocity_mps_f, position_m_f] = evaluate_polynomial(single(0.0), single(0.0), velocity_mps_f, position_m_f, timeRemaining_s_f, single(0.0));
end
end
