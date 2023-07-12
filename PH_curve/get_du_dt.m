function du_dt = get_du_dt(t, u_coeff)
%GET_DU_DT 此处显示有关此函数的摘要
%   此处显示详细说明

u0 = u_coeff(1);
u1 = u_coeff(2);
u2 = u_coeff(3);

du_dt = 2*(u0*(t-1) + u1*(1-2*t) + u2*t);
end

