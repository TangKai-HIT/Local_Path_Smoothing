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
    
    methods
        function obj = CircleSegment2D(startPt, startHeading, R, theta)
            %CIRCLESEGMENT2D 构造此类的实例
            %   R: R>0--anti-clockwise; R<0--clockwise;
            %   theta: theta>0--forward; theta<0--backward
            if nargin > 0
                obj.startPt = startPt;
                obj.startHeading = startHeading;
                rot2d = @(t) [cos(t), -sin(t); cos(t), sin(t)];
                if R>0
                    obj.endHeading = startHeading + theta;
                    
                else
                    
                end
            end
        end

        function tau = getTangentVec(phi)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

