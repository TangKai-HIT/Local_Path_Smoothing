function B_m_j = bezierBasis(j, m, u)
%BEZIERBASIS get m-th degree Bernstein polynomials
%   Detailed explanation goes here
B_m_j = nchoosek(m , j) * u.^j .* (1-u).^(m-j); 
end