classdef CircleSegment2D < handle
    %CIRCLESEGMENT2D 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        center
        theta
        startPt
        endPt
        startHeading
        endHeading
        R
    end

    properties(SetAccess = private, GetAccess = public)
        startTau;
        startNor;
        endTau;
        endNor;
    end
    
    methods
        function obj = CircleSegment2D(startPt, startHeading, R, theta)
            %CIRCLESEGMENT2D 构造此类的实例
            %   R: R>0--anti-clockwise; R<0--clockwise;
            %   theta: theta>0--head forward; theta<0--head backward
            if nargin > 0
                obj.startPt = startPt;
                obj.startHeading = startHeading;
                obj.R = R;
                obj.theta =theta;
                
                obj.endHeading = startHeading + sign(R) * theta;
                obj.startTau = rot2d(startHeading) * [1; 0];
                obj.startNor = rot2d(pi/2) * obj.startTau;
                obj.center = obj.startPt + R * obj.startNor;
                
                obj.endNor = rot2d(sign(R) * theta) * obj.startNor;
                obj.endPt = - R * obj.endNor + obj.center;
                obj.endTau = rot2d(sign(R) * theta) * obj.startTau;
            end
        end

        function [pt, tau, nor] = getPointFrenet(obj, phi)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            tau = rot2d(sign(obj.R) * phi) * obj.startTau;
            nor = rot2d(sign(obj.R) * phi) * obj.startNor;
            pt = - obj.R * nor + obj.center;
        end

        function samples = makeSamples(obj, num)
            params = linspace(0, obj.theta, num);
            samples = zeros(2, num);
            start_vec = obj.startPt - obj.center;
            dir = sign(obj.R);

            for i=1:num
                samples(:, i) = rot2d(dir * params(i)) * start_vec + obj.center;
            end
        end

    end
end

