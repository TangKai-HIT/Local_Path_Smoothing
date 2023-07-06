function u_bar = get_knotDistrib(mu, points)
%GET_KNOTDISTRIB get knot points distribution
%   input:
%       mu: 0--equally spaced; 1--cord length distribution;
%               1/2--centripetal distribution
%       points: way points for interpolation (N x dim)

N = size(points, 1);
u_bar = zeros(1, N);

if mu==0
    u_bar = 0: 1/(N-1) : 1;
elseif mu==1 || mu==1/2
    for i=2:N
        u_bar(i) = norm(points(i, :)-points(i-1, :))^mu;
    end
    denom = sum(u_bar);
    u_bar = cumsum(u_bar)/denom;
else
    u_bar=[];
    disp("invalid mu!");
end

end

