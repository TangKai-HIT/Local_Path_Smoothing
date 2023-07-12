function controlPts = get_PH_controlPts(u_coeff, v_coeff)
%GET_PH_CONTROLPTS get control points of a PH curve by assuming P0 as origin, P0P1 aligned with x-axis
%   outputs:
%       controlPts: 6 X 2

u0 = u_coeff(1);
u1 = u_coeff(2);
u2 = u_coeff(3);
v2 = v_coeff(3);

controlPts = zeros(6, 2);

controlPts(1, :) = [0, 0]; %P0
controlPts(2, :) = controlPts(1, :) + 1/5*[u0^2, 0]; %P1
controlPts(3, :) = controlPts(2, :) + 1/5*[u0*u1, 0]; %P2
controlPts(4, :) = controlPts(3, :) + 1/15*[2*u1^2 + u0*u2, u0*v2]; %P3
controlPts(5, :) = controlPts(4, :) + 1/5*[u1*u2, u1*v2]; %P4
controlPts(6, :) = controlPts(5, :) + 1/5*[u2^2 - v2^2, 2*u2*v2]; %P5
end

