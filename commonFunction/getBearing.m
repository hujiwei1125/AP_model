% loc2相对于loc1的方位，范围[0, 360)
function bearing_deg_f = getBearing(loc1_struct, loc2_struct)
%#codegen

off_x = double(loc2_struct(2) - loc1_struct(2)) * 1000;

longitude_scale = min(max(cos(deg2rad(loc2_struct(1))), 0.01), 1);

off_y = double((loc2_struct(1) - loc1_struct(1)) * 1000 / longitude_scale);

bearing_deg_f = single(90) + rad2deg(atan2(-off_y, off_x));

if bearing_deg_f < 0
    bearing_deg_f = bearing_deg_f + single(360);
end

bearing_deg_f = single(bearing_deg_f);

end