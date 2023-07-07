function dddu_ddds = get_dddu_ddds(dp_du, ddp_ddu, dddp_dddu)
%GET_DDDU_DDDS 此处显示有关此函数的摘要
%   此处显示详细说明
f_u = norm(dp_du);
df_du = dot(dp_du, ddp_ddu) / f_u;
ddf_ddu = (norm(ddp_ddu) * (dot(ddp_ddu, ddp_ddu) + dot(dp_du, dddp_dddu)) - dot(dp_du, ddp_ddu)^2) / norm(dddp_dddu);

dddu_ddds = (3 * df_du^2 - f_u * ddf_ddu) / (f_u^5);
end

