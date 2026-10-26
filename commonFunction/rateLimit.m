function output_f = rateLimit(target_f, state_f, riseRate_f, fallRate_f, dt_s_f)

delta_f = target_f - state_f;
deltaMin_f = -fallRate_f * dt_s_f;
deltaMax_f = riseRate_f * dt_s_f;
deltaLimited_f = constrain_scalar(delta_f, deltaMin_f, deltaMax_f);

output_f = state_f + deltaLimited_f;

end