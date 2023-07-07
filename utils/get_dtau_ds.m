function dtau_ds = get_dtau_ds(dp_du, ddp_ddu)
%GET_DTAU_DS 此处显示有关此函数的摘要
%   此处显示详细说明
dp_ds = get_dp_ds(dp_du);
ddp_dds = get_ddp_dds(dp_du, ddp_ddu);

dtau_ds = -dot(ddp_dds, dp_ds)/(norm(dp_ds)^3) * dp_ds + ddp_dds / norm(dp_ds);
end

