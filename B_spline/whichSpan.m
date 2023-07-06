function mid = whichSpan(u, p, u_vec)
%WHICHSPAN helper function: find span index of u
%   Output:
%       mid: span index (0~n_knot)

    n_knot = length(u_vec) - 1;
    high = n_knot - p;
    low = p;

    if abs(u-u_vec(high+1))<1e-8
        mid = high;
    else
        mid = floor((low+high)/2);

        while u<u_vec(mid+1) || u>=u_vec((mid+1)+1)
            if abs(u - u_vec((mid+1)+1))<1e-8
                mid = mid + 1;

            else
                if u>u_vec(mid+1)
                    low = mid;
                else
                    high = mid;
                end

                mid = floor((low+high)/2);
            end
            
        end
    end

end