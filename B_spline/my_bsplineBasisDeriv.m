function dB_k_j = my_bsplineBasisDeriv(j, k, p, u_vec, u)
%MY_BSPLINEBASISDERIV get k-th derivative of j-th basis with degree p
%   Input:
%       j: 1 X M, index range of basis (continuous)
%       k: order of derivatives (k <= p)
%       p: degree
%       
%   Output:
%       dB_k_j: 1 X M

if k<=p
    numBasis = length(j);
    a = zeros(k+1, k+1, numBasis);
    
    basisId = j(1) : j(end)+k;
    B_temp = zeros(size(basisId));
    dB_k_j = zeros(size(j));
    
    %Pre-compute basis
    for n = basisId
        B_temp(n - basisId(1)+1) = my_bsplineBasis(n, p-k, u_vec, u);
    end
    
    %Compute a_k
    a(1, 1, :) = 1;
    
    for l=1:numBasis
        for m=2:k+1
            % a_{k,0}
            k_Id = m-1; %index k
            denom = u_vec((j(l)+p-k_Id+1)+1) - u_vec(j(l)+1);
            if denom == 0
                a(m, 1, l) = 0;
            else
                a(m, 1, l) = a(k_Id, 1, l) / denom;
            end

            % a_{k,i} (i=1, ... ,k-1)
            for n=2:m-1
                i_Id = n-1; %index i
                denom = u_vec((j(l)+p+i_Id-k_Id+1)+1) - u_vec(j(l)+i_Id+1);
                 if denom == 0
                    a(m, n, l) = 0;
                else
                    a(m, n, l) = (a(k_Id, n, l) - a(k_Id, i_Id, l)) / denom;
                end
            end
            
            % a_{k,k}
            denom = u_vec((j(l)+p+1)+1) - u_vec(j(l)+k_Id+1);
             if denom == 0
                a(m, m, l) = 0;
            else
                a(m, m, l) = -a(k_Id, k_Id, l) / denom;
            end

        end
    end
    
    %Compute basis derivatives
    for l = 1:numBasis
        for m = 1:k+1
            i_Id = m-1;
            dB_k_j(l) = dB_k_j(l) + a(k+1, m, l) * B_temp(j(l)+i_Id-basisId(1)+1);
        end
        dB_k_j(l) = factorial(p)/factorial(p-k) * dB_k_j(l);
    end

else
    disp("invalid k! (k<=p)");
    dB_k_j = [];
end