function v_t = get_v_t(t, v_coeff)
%GET_V_T 此处显示有关此函数的摘要
%   此处显示详细说明

v0 = v_coeff(1);
v1 = v_coeff(2);
v2 = v_coeff(3);

v_t = v0*(1-t).^2 + 2*v1*(1-t).*t + v2*t.^2;
end

