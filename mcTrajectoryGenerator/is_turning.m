function turning_b = is_turning(trajPositionNEU_m_f, trajVelocityNEU_mps_f, targetNEU_m_f, targetAcceptanceRadius_m_f, turnMinSpeed_mps_f, turnCosThreshold_f, smallValue_f)
% 判断当前水平轨迹是否进入“速度方向与目标方向明显不一致”的转弯状态。
velocityNE_mps_f = trajVelocityNEU_mps_f(1:2);
positionToTargetNE_m_f = targetNEU_m_f(1:2) - trajPositionNEU_m_f(1:2);
velocityDirectionNE_f = unit_or_zero(velocityNE_mps_f, smallValue_f);
targetDirectionNE_f = unit_or_zero(positionToTargetNE_m_f, smallValue_f);
directionCosine_f = dot(velocityDirectionNE_f, targetDirectionNE_f);

turning_b = norm(velocityNE_mps_f) > turnMinSpeed_mps_f && directionCosine_f < turnCosThreshold_f && norm(positionToTargetNE_m_f) > targetAcceptanceRadius_m_f;
end
