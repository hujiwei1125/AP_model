function T1_s_f = compute_T1_given_total_time(totalTime_s_f, accelerationInitial_mps2_f, deltaVelocity_mps_f, jerkStage1_mps3_f, maxAcceleration_mps2_f)
% 在给定总持续时间T=T1+T2+T3时，反算第一阶段持续时间。
quadraticA_f = -jerkStage1_mps3_f;
quadraticB_f = jerkStage1_mps3_f * totalTime_s_f - accelerationInitial_mps2_f;
discriminant_f = totalTime_s_f * totalTime_s_f * jerkStage1_mps3_f * jerkStage1_mps3_f + ...
    single(2.0) * totalTime_s_f * accelerationInitial_mps2_f * jerkStage1_mps3_f - accelerationInitial_mps2_f * accelerationInitial_mps2_f - ...
    single(4.0) * jerkStage1_mps3_f * deltaVelocity_mps_f;

if discriminant_f < single(0.0)
    T1_s_f = single(0.0);
    return;
end

sqrtDiscriminant_f = sqrt(discriminant_f);
inverseDenominator_f = single(1.0) / (single(2.0) * quadraticA_f);
T1Positive_s_f = max((-quadraticB_f + sqrtDiscriminant_f) * inverseDenominator_f, single(0.0));
T1Negative_s_f = max((-quadraticB_f - sqrtDiscriminant_f) * inverseDenominator_f, single(0.0));
T3Positive_s_f = accelerationInitial_mps2_f / jerkStage1_mps3_f + T1Positive_s_f;
T3Negative_s_f = accelerationInitial_mps2_f / jerkStage1_mps3_f + T1Negative_s_f;

T1_s_f = single(0.0);
if T1Positive_s_f >= single(0.0) && T3Positive_s_f >= single(0.0) && T1Positive_s_f + T3Positive_s_f <= totalTime_s_f
    T1_s_f = T1Positive_s_f;
elseif T1Negative_s_f >= single(0.0) && T3Negative_s_f >= single(0.0) && T1Negative_s_f + T3Negative_s_f <= totalTime_s_f
    T1_s_f = T1Negative_s_f;
end

T1_s_f = saturate_T1_for_acceleration(accelerationInitial_mps2_f, jerkStage1_mps3_f, T1_s_f, maxAcceleration_mps2_f);
end
