classdef LineSegment < handle
    %LINESEGMENT 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        startPt
        endPt
        len
        tau
    end
    
    methods
        function obj = LineSegment(startPt,endPt)
            %LINESEGMENT 构造此类的实例
            %   此处显示详细说明
            if nargin > 0
                obj.startPt = startPt;
                obj.endPt = endPt;
                obj.len = norm(startPt - endPt);
                obj.tau = (endPt - startPt)/obj.len;
            end
        end

        function pt = getPointFromEnd(obj, dis)
            pt = obj.endPt + (obj.startPt - obj.endPt)/obj.len * dis;
        end

        function pt = getPointFromStart(obj, dis)
            pt = obj.startPt + (obj.endPt - obj.startPt)/obj.len * dis;
        end
    end
end

