function T3_s_f = compute_T3(T1_s_f, accelerationInitial_mps2_f, jerkStage1_mps3_f)
% 第三阶段使用与第一阶段相反的jerk，使最终加速度回到0。
T3_s_f = max(accelerationInitial_mps2_f / jerkStage1_mps3_f + T1_s_f, 0.0);
end
