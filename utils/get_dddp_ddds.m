function dddp_ddds = get_dddp_ddds(dp_du, ddp_ddu, dddp_dddu)
%GET_DDDP_DDDS 此处显示有关此函数的摘要
%   此处显示详细说明
du_ds = get_du_ds(dp_du);
ddu_dds = get_ddu_dds(dp_du, ddp_ddu);
dddu_ddds = get_dddu_ddds(dp_du, ddp_ddu, dddp_dddu);

dddp_ddds = dddp_dddu * du_ds^3 + 3 * ddp_ddu * du_ds * ddu_dds + dp_du * dddu_ddds;
end

