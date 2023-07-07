function equal_CtrlPts = bezierSurface_isoParamCurve(ctrl_pts, fixParam, paramName)
%BEZIERSURFACE_ISOPARAMCURVE get isoparametric bezier curve control points on the surface
%   Inputs:
%       fixParam: value of the fixed parameter
%       paramName: 'u' or 'v'

[N, M, ~] = size(ctrl_pts);
p = N-1;  q = M-1;

switch paramName
    case 'u'
        %Get v-parametric curve control points on u
        B_u_j = zeros(1, N);
        for i=1:N
            B_u_j(i) = bezierBasis(i-1, p, fixParam);
        end
        equal_CtrlPts = tensorprod(ctrl_pts, B_u_j, 1, 2); 

    case 'v'
        %Get u-parametric curve control points on v
        B_v_j = zeros(1, M);
        for i=1:M
            B_v_j(i) = bezierBasis(i-1, q, fixParam);
        end
        equal_CtrlPts = tensorprod(ctrl_pts, B_v_j, 2, 2); 
        
    otherwise
        disp("input u or v!");
        equal_CtrlPts = [];
end