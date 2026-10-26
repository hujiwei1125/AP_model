function [T1OutNEU_s_f, T2OutNEU_s_f, T3OutNEU_s_f] = synchronize_trajectory_time_neu(T1InNEU_s_f, T2InNEU_s_f, T3InNEU_s_f, accelerationNEU_mps2_f, velocityNEU_mps_f, velocitySetpointNEU_mps_f, directionNEU_f, maxAccelerationNEU_mps2_f, maxJerkNEU_mps3_f, smallValue_f)
%#codegen
% 将N/E/U三轴S曲线同步到相同的总持续时间。
%
% 算法：
% 1. 各轴先按自身jerk/acc约束计算最短时间T1/T2/T3；
% 2. 取三轴中最长的总时间作为同步目标时间；
% 3. 对其余需要改变速度的轴，在保持各自jerk/acc上限不变的前提下，
%    反算新的T1/T2/T3，使其总时间等于同步目标时间；
% 4. 对速度目标已经等于当前速度且加速度近似为0的静止轴，不强行构造无意义的S曲线。
%
% 说明：
% 三轴可以具有不同的最大jerk和最大加速度。时间同步不是把约束改成相同，
% 而是让较快轴使用更缓的S曲线，在与最慢轴相同的时间内完成速度变化。

T1OutNEU_s_f = T1InNEU_s_f;
T2OutNEU_s_f = T2InNEU_s_f;
T3OutNEU_s_f = T3InNEU_s_f;

synchronizedTime_s_f = single(0.0);
longestAxis_i = int32(0);

for axisIdx_i = 1:3
    totalTime_s_f = T1InNEU_s_f(axisIdx_i) + T2InNEU_s_f(axisIdx_i) + T3InNEU_s_f(axisIdx_i);
    if totalTime_s_f > synchronizedTime_s_f
        synchronizedTime_s_f = totalTime_s_f;
        longestAxis_i = int32(axisIdx_i);
    end
end

if synchronizedTime_s_f <= smallValue_f || longestAxis_i == 0
    return;
end

for axisIdx_i = 1:3
    if axisIdx_i == longestAxis_i
        continue;
    end

    totalTime_s_f = T1InNEU_s_f(axisIdx_i) + T2InNEU_s_f(axisIdx_i) + T3InNEU_s_f(axisIdx_i);
    if totalTime_s_f >= synchronizedTime_s_f - smallValue_f
        continue;
    end

    deltaVelocity_mps_f = velocitySetpointNEU_mps_f(axisIdx_i) - velocityNEU_mps_f(axisIdx_i);
    axisNeedsMotion_b = abs(deltaVelocity_mps_f) > smallValue_f || abs(accelerationNEU_mps2_f(axisIdx_i)) > smallValue_f;
    if ~axisNeedsMotion_b
        continue;
    end

    jerkStage1_mps3_f = directionNEU_f(axisIdx_i) * maxJerkNEU_mps3_f(axisIdx_i);
    if abs(jerkStage1_mps3_f) <= smallValue_f
        continue;
    end

    [T1New_s_f, T2New_s_f, T3New_s_f] = update_durations_given_total_time(synchronizedTime_s_f, accelerationNEU_mps2_f(axisIdx_i), deltaVelocity_mps_f, jerkStage1_mps3_f, maxAccelerationNEU_mps2_f(axisIdx_i), smallValue_f);

    newTotalTime_s_f = T1New_s_f + T2New_s_f + T3New_s_f;
    if newTotalTime_s_f > smallValue_f
        T1OutNEU_s_f(axisIdx_i) = T1New_s_f;
        T2OutNEU_s_f(axisIdx_i) = T2New_s_f;
        T3OutNEU_s_f(axisIdx_i) = T3New_s_f;
    end
end
end
