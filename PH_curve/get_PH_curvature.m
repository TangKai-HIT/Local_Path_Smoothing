function kappa = get_PH_curvature(t, u_coeff, v_coeff)
%GET_PH_CURVATURE get curvature of a PH curve at t
%   此处显示详细说明

u = get_u_t(t, u_coeff);
v = get_v_t(t, v_coeff);

du = get_du_dt(t, u_coeff);
dv = get_dv_dt(t, v_coeff);

kappa = 2*(u .* dv - du .* v) ./ ((u.^2 + v.^2).^2);
end

