function angle_deg_d = cldWrap180Double(angle_deg_d)
%#codegen
%
% 将double型角度限制到[-180, 180)

angle_deg_d = mod(angle_deg_d + 180.0, 360.0) - 180.0;

end