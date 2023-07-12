function u_t = get_u_t(t, u_coeff)
%GET_U_T 此处显示有关此函数的摘要
%   此处显示详细说明

u0 = u_coeff(1);
u1 = u_coeff(2);
u2 = u_coeff(3);

u_t = u0*(1-t).^2 + 2*u1*(1-t).*t + u2*t.^2;
end

