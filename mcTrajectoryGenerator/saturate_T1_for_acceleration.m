function T1New_s_f = saturate_T1_for_acceleration(accelerationInitial_mps2_f, jerkStage1_mps3_f, T1_s_f, maxAcceleration_mps2_f)
% 调整第一阶段持续时间，使第一阶段末端加速度不超过允许范围。
accelerationAtT1_mps2_f = accelerationInitial_mps2_f + jerkStage1_mps3_f * T1_s_f;
T1New_s_f = T1_s_f;
if accelerationAtT1_mps2_f > maxAcceleration_mps2_f
    T1New_s_f = (maxAcceleration_mps2_f - accelerationInitial_mps2_f) / jerkStage1_mps3_f;
elseif accelerationAtT1_mps2_f < -maxAcceleration_mps2_f
    T1New_s_f = (-maxAcceleration_mps2_f - accelerationInitial_mps2_f) / jerkStage1_mps3_f;
end
end
