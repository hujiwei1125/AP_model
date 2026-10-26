function output_f = constrain_scalar(input_f, lowerBound_f, upperBound_f)
% 将标量限制在指定上下界之间。
output_f = min(max(input_f, lowerBound_f), upperBound_f);
end
