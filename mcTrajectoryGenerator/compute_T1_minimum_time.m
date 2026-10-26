function T1_s_f = compute_T1_minimum_time(accelerationInitial_mps2_f, deltaVelocity_mps_f, jerkStage1_mps3_f, maxAcceleration_mps2_f, smallValue_f)
% 在T2=0的最短时间情形下求第一阶段持续时间。
% 先求二次方程的两个候选根，再选择能够同时保证T1>=0、T3>=0的根。

discriminant_f = single(2.0) * accelerationInitial_mps2_f * accelerationInitial_mps2_f + single(4.0) * jerkStage1_mps3_f * deltaVelocity_mps_f;
if discriminant_f < single(0.0) || abs(jerkStage1_mps3_f) <= smallValue_f
    T1_s_f = single(0.0);
    return;
end

sqrtDiscriminant_f = sqrt(discriminant_f);
T1Positive_s_f = (-accelerationInitial_mps2_f + 0.5 * sqrtDiscriminant_f) / jerkStage1_mps3_f;
T1Negative_s_f = (-accelerationInitial_mps2_f - 0.5 * sqrtDiscriminant_f) / jerkStage1_mps3_f;
T3Positive_s_f = accelerationInitial_mps2_f / jerkStage1_mps3_f + T1Positive_s_f;
T3Negative_s_f = accelerationInitial_mps2_f / jerkStage1_mps3_f + T1Negative_s_f;

T1_s_f = single(0.0);
if T1Positive_s_f >= single(0.0) && T3Positive_s_f >= single(0.0)
    T1_s_f = T1Positive_s_f;
elseif T1Negative_s_f >= single(0.0) && T3Negative_s_f >= single(0.0)
    T1_s_f = T1Negative_s_f;
end

T1_s_f = saturate_T1_for_acceleration(accelerationInitial_mps2_f, jerkStage1_mps3_f, T1_s_f, maxAcceleration_mps2_f);
T1_s_f = max(T1_s_f, single(0.0));
end
