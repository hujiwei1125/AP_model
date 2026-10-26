function maxSpeedU_mps_f = get_max_z_speed(trajectoryU_m_f, targetU_m_f, nextTargetU_m_f, maxJerkU_mps3_f, maxAccelU_mps2_f, maxVelUp_mps_f, maxVelDown_mps_f, verticalAcceptanceRadius_m_f, smallValue_f)
% 根据当前高度、目标高度和下一目标高度计算允许的垂向速度幅值。
% 若下一航段仍沿相同垂向方向运动，则允许带一定速度通过当前航点；
% 若下一航段反向或没有明显高度变化，则当前航点速度趋向0。

currentDirection_f = sign(targetU_m_f - trajectoryU_m_f);
if currentDirection_f > single(0.0)
    directionalSpeedLimit_mps_f = maxVelUp_mps_f;
else
    directionalSpeedLimit_mps_f = maxVelDown_mps_f;
end

arrivalSpeed_mps_f = single(0.0);
nextAltitudeDifferent_b = abs(nextTargetU_m_f - targetU_m_f) > verticalAcceptanceRadius_m_f;
nextDirection_f = sign(nextTargetU_m_f - targetU_m_f);

if nextAltitudeDifferent_b && currentDirection_f ~= 0.0 && nextDirection_f == currentDirection_f
    nextDistance_m_f = abs(nextTargetU_m_f - targetU_m_f);
    arrivalSpeed_mps_f = compute_max_speed_from_distance(maxJerkU_mps3_f, maxAccelU_mps2_f, nextDistance_m_f, 0.0, smallValue_f);
    arrivalSpeed_mps_f = min(arrivalSpeed_mps_f, directionalSpeedLimit_mps_f);
end

distanceToTarget_m_f = abs(targetU_m_f - trajectoryU_m_f);
maxSpeedFromDistance_mps_f = compute_max_speed_from_distance(maxJerkU_mps3_f, maxAccelU_mps2_f, distanceToTarget_m_f, arrivalSpeed_mps_f, smallValue_f);
maxSpeedU_mps_f = min(directionalSpeedLimit_mps_f, maxSpeedFromDistance_mps_f);
end
