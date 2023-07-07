function ddp_dds = get_ddp_dds(dp_du, ddp_ddu)
%GET_DDP_DDS 此处显示有关此函数的摘要
%   此处显示详细说明
du_ds = get_du_ds(dp_du);
ddu_dds = get_ddu_dds(dp_du, ddp_ddu);

ddp_dds = ddp_ddu * du_ds^2 + dp_du * ddu_dds;
end

