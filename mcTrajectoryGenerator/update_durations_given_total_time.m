function [T1_s_f, T2_s_f, T3_s_f] = update_durations_given_total_time(totalTime_s_f, accelerationInitial_mps2_f, deltaVelocity_mps_f, jerkStage1_mps3_f, maxAcceleration_mps2_f, smallValue_f)
% 在指定总持续时间下重新计算S曲线三个阶段的持续时间。
if abs(jerkStage1_mps3_f) <= smallValue_f
    T1_s_f = single(0.0);
    T2_s_f = single(0.0);
    T3_s_f = single(0.0);
    return;
end

T1_s_f = compute_T1_given_total_time(totalTime_s_f, accelerationInitial_mps2_f, deltaVelocity_mps_f, jerkStage1_mps3_f, maxAcceleration_mps2_f);
T3_s_f = compute_T3(T1_s_f, accelerationInitial_mps2_f, jerkStage1_mps3_f);
T2_s_f = max(totalTime_s_f - T1_s_f - T3_s_f, single(0.0));
end
