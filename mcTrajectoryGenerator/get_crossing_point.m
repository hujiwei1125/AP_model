function crossingPointNEU_m_f = get_crossing_point(trajPositionNEU_m_f, trajVelocityNEU_mps_f, waypointsNEU_m_f, targetAptRadius_m_f, minLookaheadDist_m_f, ...
    turnMinSpeed_mps_f, turnCosThreshold_f, smallValue_f)
% 根据当前状态选择水平追踪点。
% 非转弯状态：直接追踪当前目标航点。
% 转弯状态：在“上一航点->当前目标航点”直线上取一个前视点，减小转弯时回拉目标点的现象。

targetNEU_m_f = waypointsNEU_m_f(:,2);
if ~is_turning(trajPositionNEU_m_f, trajVelocityNEU_mps_f, targetNEU_m_f, targetAptRadius_m_f, turnMinSpeed_mps_f, turnCosThreshold_f, smallValue_f)
    crossingPointNEU_m_f = targetNEU_m_f;
    return;
end

pathVectorNE_m_f = waypointsNEU_m_f(1:2,2) - waypointsNEU_m_f(1:2,1);
pathDirectionNE_f = unit_or_zero(pathVectorNE_m_f, smallValue_f);
if norm(pathDirectionNE_f) <= smallValue_f
    crossingPointNEU_m_f = targetNEU_m_f;
    return;
end

previousToTrajectoryNE_m_f = trajPositionNEU_m_f(1:2) - waypointsNEU_m_f(1:2,1);
projectionLength_m_f = dot(previousToTrajectoryNE_m_f, pathDirectionNE_f);
closestPointNE_m_f = waypointsNEU_m_f(1:2,1) + projectionLength_m_f * pathDirectionNE_f;
crossTrackError_m_f = norm(closestPointNE_m_f - trajPositionNEU_m_f(1:2));
lookaheadDistance_m_f = max(targetAptRadius_m_f, minLookaheadDist_m_f);
alongTrackDistance_m_f = single(0.0);

if lookaheadDistance_m_f > crossTrackError_m_f
    alongTrackDistance_m_f = sqrt(max(lookaheadDistance_m_f * lookaheadDistance_m_f - crossTrackError_m_f * crossTrackError_m_f, single(0.0)));
end

crossingPointNEU_m_f = targetNEU_m_f;
crossingPointNEU_m_f(1:2) = closestPointNE_m_f + alongTrackDistance_m_f * pathDirectionNE_f;
end
