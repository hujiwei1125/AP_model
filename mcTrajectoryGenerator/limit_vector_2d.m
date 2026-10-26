function outputVector_f = limit_vector_2d(inputVector_f, maxNorm_f, smallValue_f)
% 将二维向量的模限制在maxNorm_f以内，同时保持原方向不变。
inputNorm_f = norm(inputVector_f);
if inputNorm_f > maxNorm_f && inputNorm_f > smallValue_f
    outputVector_f = inputVector_f * maxNorm_f / inputNorm_f;
else
    outputVector_f = inputVector_f;
end
end
