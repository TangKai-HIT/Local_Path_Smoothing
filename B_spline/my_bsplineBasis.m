function B_j = my_bsplineBasis(j, p, u_vec, u)
%MY_BSPLINEBASIS evaluate j-th basis of order p at u
%   Input:
%       j: index of basis function
%       p: order of polynomial
%       u_vec: knot vector
%       u: input vector 
%   Output:
%       B_j: vector, j-th basis function evaluated at u (same size as u)

B_j = zeros(size(u));
j=j+1; %shift the index (index>=1)

if p>0
    l_denom = u_vec(j+p) - u_vec(j);
    r_denom = u_vec(j+p+1) - u_vec(j+1);

    if l_denom~=0
        left = (u - u_vec(j)) ./ l_denom .* my_bsplineBasis(j-1, p-1, u_vec, u);
    else
        left = 0;
    end

    if r_denom~=0
        right = (u_vec(j+p+1) - u) ./ r_denom .* my_bsplineBasis(j, p-1, u_vec, u);
    else
        right = 0;
    end
    
    B_j = left + right;
else
    if u_vec(j+1) < u_vec(end)
        B_j(u_vec(j)<=u & u<u_vec(j+1)) = 1;
    else
        B_j(u_vec(j)<=u) = 1;
    end
end

end

