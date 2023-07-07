function ddu_dds = get_ddu_dds(dp_du, ddp_ddu)
%GET_DDU_DDS 此处显示有关此函数的摘要
%   此处显示详细说明
f_u = norm(dp_du);
df_du = dot(dp_du, ddp_ddu) / f_u;

ddu_dds = -df_du/(f_u^3);
end

