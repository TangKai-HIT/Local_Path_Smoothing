function kappa = get_kappa(dp_du,ddp_ddu)
%GET_KAPPA 此处显示有关此函数的摘要
%   此处显示详细说明
if length(dp_du) == 3
    kappa = cross(dp_du, ddp_ddu) / norm(dp_du)^3;
else
    kappa = dot([0, -1; 1, 0] * dp_du, ddp_ddu) / norm(dp_du)^3;
end
end

