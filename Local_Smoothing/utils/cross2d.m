function crossprod = cross2d(A, B)
%CROSS2D 此处显示有关此函数的摘要
%   此处显示详细说明
W = [0, -1; 1, 0];
crossprod = dot(W * A, B);
end

