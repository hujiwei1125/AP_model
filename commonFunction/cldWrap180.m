function d_wrap180 = cldWrap180(d)
%#codegen
%
% 将角度限制到 [-180, 180)

SCALE = 100000000;

v = (d + 180) * SCALE;

v = rem(rem(v, 360 * SCALE) + 360 * SCALE, 360 * SCALE);

d_wrap180 = single(v / SCALE - 180);

end