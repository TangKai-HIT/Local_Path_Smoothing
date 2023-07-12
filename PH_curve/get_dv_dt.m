function dv_dt = get_dv_dt(t, v_coeff)
%GET_DV_DT 此处显示有关此函数的摘要
%   此处显示详细说明

v0 = v_coeff(1);
v1 = v_coeff(2);
v2 = v_coeff(3);

dv_dt = 2*(v0*(t-1) + v1*(1-2*t) + v2*t);
end

