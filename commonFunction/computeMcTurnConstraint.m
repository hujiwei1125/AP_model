function [judgeDist_m_f, passSpeed_mps_f] = computeMcTurnConstraint(preWpNE_m_f, curWpNE_m_f, nextWpNE_m_f, ...
    maxRouteSpeed_mps_f, maxLatAcc_mps2_f, minJudgeDist_m_f, maxJudgeDist_m_f, segmentLimitRatio_f, smallValue_f)
%#codegen
%
% 旋翼快速航点转弯约束计算
%
% 输出：
% judgeDist_m_f
%   当前航点提前分界面距离L
%
% passSpeed_mps_f
%   当前航点允许通过速度
%
% 原理：
%
% R = V^2 / a
%
% L = R * tan(theta / 2)
%
% 首先按最大航线速度计算理论提前距离。
%
% 如果理论提前距离大于实际允许提前距离，则根据实际提前距离
% 反算允许通过速度：
%
% Vpass = sqrt(a * L / tan(theta / 2))
%
% 因此maxRouteSpeed_mps_f只是速度上限，而不是固定通过速度。

zero_f = single(0.0);
one_f = single(1.0);

% judgeDist_m_f = zero_f;
% passSpeed_mps_f = zero_f;

%% 参数保护

maxRouteSpeed_mps_f = max(maxRouteSpeed_mps_f, zero_f);
maxLatAcc_mps2_f = max(maxLatAcc_mps2_f, zero_f);

minJudgeDist_m_f = max(minJudgeDist_m_f, zero_f);
maxJudgeDist_m_f = max(maxJudgeDist_m_f, minJudgeDist_m_f);

segmentLimitRatio_f = min(max(segmentLimitRatio_f, zero_f), single(0.49));

%% 当前航段和下一航段

inVecNE_m_f = curWpNE_m_f - preWpNE_m_f;
outVecNE_m_f = nextWpNE_m_f - curWpNE_m_f;

inLength_m_f = norm(inVecNE_m_f);
outLength_m_f = norm(outVecNE_m_f);

%% 当前航段无效

if inLength_m_f <= smallValue_f

    judgeDist_m_f = zero_f;
    passSpeed_mps_f = zero_f;

    return;

end

%% 无有效下一航段

% 没有下一航段时，相当于当前快速航点之后没有可继续通过的航段。
% 按末端速度0处理。

if outLength_m_f <= smallValue_f

    geometryMaxDist_m_f = segmentLimitRatio_f * inLength_m_f;

    effectiveMaxDist_m_f = min(maxJudgeDist_m_f, geometryMaxDist_m_f);

    judgeDist_m_f = min(minJudgeDist_m_f, effectiveMaxDist_m_f);

    passSpeed_mps_f = zero_f;

    return;

end

%% 航段单位方向

inDirNE_f = inVecNE_m_f / inLength_m_f;
outDirNE_f = outVecNE_m_f / outLength_m_f;

%% 计算实际转角

% theta：
%
% 0 deg   -> 直线
% 90 deg  -> 直角转弯
% 180 deg -> 掉头

turnCos_f = dot(inDirNE_f, outDirNE_f);

turnCos_f = min(max(turnCos_f, -one_f), one_f);

turnAngle_rad_f = acos(turnCos_f);

%% 相邻航段长度限制

geometryMaxDist_m_f = segmentLimitRatio_f * min(inLength_m_f, outLength_m_f);

effectiveMaxDist_m_f = min(maxJudgeDist_m_f, geometryMaxDist_m_f);

if effectiveMaxDist_m_f <= smallValue_f

    judgeDist_m_f = zero_f;
    passSpeed_mps_f = zero_f;

    return;

end

%% 基本为直线

if turnAngle_rad_f <= single(1.0e-3)% || maxRouteSpeed_mps_f <= smallValue_f

    judgeDist_m_f = min(minJudgeDist_m_f, effectiveMaxDist_m_f);

    passSpeed_mps_f = maxRouteSpeed_mps_f;

    return;

end

%% 半转角

halfTurnAngle_rad_f = min(single(0.5) * turnAngle_rad_f, single(89.0 * pi / 180.0));

tanHalfTurn_f = tan(halfTurnAngle_rad_f);

if tanHalfTurn_f <= smallValue_f

    judgeDist_m_f = min(minJudgeDist_m_f, effectiveMaxDist_m_f);

    passSpeed_mps_f = maxRouteSpeed_mps_f;

    return;

end

%% 横向加速度不可用

% if maxLatAcc_mps2_f <= smallValue_f
% 
%     judgeDist_m_f = effectiveMaxDist_m_f;
%     passSpeed_mps_f = zero_f;
% 
%     return;
% 
% end

%% 最大航线速度下需要的转弯提前距离

maxTurnRadius_m_f = maxRouteSpeed_mps_f * maxRouteSpeed_mps_f / maxLatAcc_mps2_f;

requiredJudgeDist_m_f = maxTurnRadius_m_f * tanHalfTurn_f;

%% 实际提前距离

if effectiveMaxDist_m_f >= minJudgeDist_m_f

    judgeDist_m_f = min(max(requiredJudgeDist_m_f, minJudgeDist_m_f), effectiveMaxDist_m_f);

else

    judgeDist_m_f = effectiveMaxDist_m_f;

end

%% 根据实际提前距离反算航点通过速度

turnRadius_m_f = judgeDist_m_f / tanHalfTurn_f;

passSpeed_mps_f = sqrt(max(maxLatAcc_mps2_f * turnRadius_m_f, zero_f));

passSpeed_mps_f = min(passSpeed_mps_f, maxRouteSpeed_mps_f);

end