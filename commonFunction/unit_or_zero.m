function unitVector_f = unit_or_zero(vector_f, smallValue_f)
% 计算二维向量的单位方向。
% 当向量模过小时输出零向量，避免除零和数值放大。
vectorNorm_f = norm(vector_f);
if vectorNorm_f > smallValue_f
    unitVector_f = vector_f / vectorNorm_f;
else
    unitVector_f = vector_f * 0.0;
end
end
