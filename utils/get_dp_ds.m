function dp_ds = get_dp_ds(dp_du)
%GET_DP_DS 此处显示有关此函数的摘要
%   此处显示详细说明
du_ds = get_du_ds(dp_du);
dp_ds = dp_du * du_ds;
end

