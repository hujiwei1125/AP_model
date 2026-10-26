clear simple6DofPlant
initSimple6DofPlant

simTime_s_f = single(100.0);
time_s_f = single((0:double(simple6DofDt_s_f):double(simTime_s_f))');
sampleNum_u32 = uint32(numel(time_s_f));

posLog_m_f = single(zeros(double(sampleNum_u32), 3));
velLog_mps_f = single(zeros(double(sampleNum_u32), 3));
attLog_deg_f = single(zeros(double(sampleNum_u32), 3));
rateLog_dps_f = single(zeros(double(sampleNum_u32), 3));
tasLog_mps_f = single(zeros(double(sampleNum_u32), 1));
phaseLog_u8 = uint8(zeros(double(sampleNum_u32), 1));

apCmd_struct = struct('vxCmd_mps_f', single(0.0), 'vyCmd_mps_f', single(0.0), 'vuCmd_mps_f', single(0.0), 'rollCmd_deg_f', single(0.0), 'pitchCmd_deg_f', single(0.0), 'yawCmd_deg_f', single(0.0), 'tiltEnable_b', false, 'apRunState_b', true, 'apCmdType_u8', uint8(0));

for idx_u32 = uint32(1):sampleNum_u32
    curTime_s_f = time_s_f(double(idx_u32));
    flightPhase_u8 = uint8(2);

    apCmd_struct.vxCmd_mps_f = single(0.0);
    apCmd_struct.vyCmd_mps_f = single(0.0);
    apCmd_struct.vuCmd_mps_f = single(0.0);
    apCmd_struct.rollCmd_deg_f = single(0.0);
    apCmd_struct.pitchCmd_deg_f = single(0.0);
    apCmd_struct.yawCmd_deg_f = single(0.0);
    apCmd_struct.tiltEnable_b = false;

    if curTime_s_f < single(10.0)
        apCmd_struct.vuCmd_mps_f = single(2.0);
    elseif curTime_s_f < single(25.0)
        apCmd_struct.vxCmd_mps_f = single(8.0);
        apCmd_struct.yawCmd_deg_f = single(45.0);
    elseif curTime_s_f < single(35.0)
        flightPhase_u8 = uint8(3);
        apCmd_struct.tiltEnable_b = true;
        apCmd_struct.vxCmd_mps_f = single(8.0) + single(3.2) * (curTime_s_f - single(25.0));
        apCmd_struct.rollCmd_deg_f = single(15.0);
    elseif curTime_s_f < single(60.0)
        flightPhase_u8 = uint8(4);
        apCmd_struct.vxCmd_mps_f = single(40.0);
        apCmd_struct.rollCmd_deg_f = single(20.0);
    elseif curTime_s_f < single(70.0)
        flightPhase_u8 = uint8(4);
        apCmd_struct.vxCmd_mps_f = single(40.0);
    elseif curTime_s_f < single(85.0)
        flightPhase_u8 = uint8(3);
        apCmd_struct.tiltEnable_b = true;
        apCmd_struct.vxCmd_mps_f = max(single(40.0) - single(40.0 / 15.0) * (curTime_s_f - single(70.0)), single(0.0));
    else
        apCmd_struct.vuCmd_mps_f = single(-1.0);
    end

    reset_b = idx_u32 == uint32(1);

    [nav_struct, posNEU_m_f, velNEU_mps_f, ~, attEuler_deg_f, rateBody_dps_f] = simple6DofPlant(apCmd_struct, flightPhase_u8, reset_b, simple6DofDt_s_f, simple6DofInitState_f, simple6DofRefLat_deg_d, simple6DofRefLon_deg_d, simple6DofRefAlt_m_f, simple6DofGroundAlt_m_f);

    posLog_m_f(double(idx_u32), :) = posNEU_m_f';
    velLog_mps_f(double(idx_u32), :) = velNEU_mps_f';
    attLog_deg_f(double(idx_u32), :) = attEuler_deg_f';
    rateLog_dps_f(double(idx_u32), :) = rateBody_dps_f';
    tasLog_mps_f(double(idx_u32)) = nav_struct.tas_mps_f;
    phaseLog_u8(double(idx_u32)) = flightPhase_u8;
end

figure('Name', 'Simple 6DoF Plant Demo', 'Color', 'w');

subplot(2, 2, 1);
plot(posLog_m_f(:, 2), posLog_m_f(:, 1), 'LineWidth', 1.2);
axis equal;
grid on;
xlabel('East / m');
ylabel('North / m');
title('Horizontal trajectory');

subplot(2, 2, 2);
plot(time_s_f, posLog_m_f(:, 3), 'LineWidth', 1.2);
grid on;
xlabel('Time / s');
ylabel('Up position / m');
title('Altitude relative to reference');

subplot(2, 2, 3);
plot(time_s_f, velLog_mps_f, 'LineWidth', 1.0);
hold on;
plot(time_s_f, tasLog_mps_f, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('Time / s');
ylabel('Speed / m/s');
legend('V_N', 'V_E', 'V_U', 'TAS', 'Location', 'best');
title('Velocity');

subplot(2, 2, 4);
plot(time_s_f, attLog_deg_f, 'LineWidth', 1.0);
grid on;
xlabel('Time / s');
ylabel('Angle / deg');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
title('Attitude');

figure('Name', 'Simple 6DoF Phase and Body Rate', 'Color', 'w');

subplot(2, 1, 1);
stairs(time_s_f, phaseLog_u8, 'LineWidth', 1.1);
grid on;
ylabel('Flight phase');
ylim([0, 5]);

subplot(2, 1, 2);
plot(time_s_f, rateLog_dps_f, 'LineWidth', 1.0);
grid on;
xlabel('Time / s');
ylabel('Rate / deg/s');
legend('p', 'q', 'r', 'Location', 'best');
