function d_wrap360 = cldWrap360(d)
%#codegen
%
% 功能：
% 将输入角度限制到 [0, 360) 范围。
%
% 输入：
% d：角度，deg
%
% 输出：
% d_wrap360：归一化后的角度，deg

SCALE = 100000000;

v = d * SCALE;

v = rem(rem(v, 360 * SCALE) + 360 * SCALE, 360 * SCALE);

d_wrap360 = v / SCALE;

end