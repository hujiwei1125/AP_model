function T2_s_f = compute_T2_from_velocity(T1_s_f, T3_s_f, accelerationInitial_mps2_f, deltaVelocity_mps_f, jerkStage1_mps3_f, smallValue_f)
% 根据总速度增量约束，反算第二阶段匀加速度持续时间。
accelerationStage2_mps2_f = accelerationInitial_mps2_f + jerkStage1_mps3_f * T1_s_f;
T2_s_f = single(0.0);

if abs(accelerationStage2_mps2_f) > smallValue_f
    numerator_f = single(-0.5) * T1_s_f * T1_s_f * jerkStage1_mps3_f - T1_s_f * T3_s_f * jerkStage1_mps3_f - ...
        T1_s_f * accelerationInitial_mps2_f + single(0.5) * T3_s_f * T3_s_f * jerkStage1_mps3_f - T3_s_f * accelerationInitial_mps2_f + ...
        deltaVelocity_mps_f;
    T2_s_f = numerator_f / accelerationStage2_mps2_f;
end

T2_s_f = max(T2_s_f, single(0.0));
end
