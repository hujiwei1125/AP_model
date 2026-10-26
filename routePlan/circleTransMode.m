function mode_u8 = circleTransMode(locBegin, locEnd, bearBegin, bearEnd)
%#codegen
% mode_u8：1(LSL)、2(RSR)、3(LSR)、4(RSL)
% 这里计算出来的旋转方向基于E-N坐标系：横轴为E，纵轴为N。
%
% 外部依赖：getBearing、cldWrap180。

bear_AB = getBearing(locBegin, locEnd);

if cldWrap180(bear_AB - bearBegin) >= 0
    if cldWrap180(bearEnd - bear_AB) >= 0
        mode_u8 = uint8(1); % 将其转换成N-E坐标系下的旋转方向
    else
        mode_u8 = uint8(3);
    end
else
    if cldWrap180(bearEnd - bear_AB) >= 0
        mode_u8 = uint8(4); % 将其转换成N-E坐标系下的旋转方向
    else
        mode_u8 = uint8(2);
    end
end
end
