classdef JOINT < handle
    properties
        point
        refFrame
        body1
        body2
        constraints
    end
    
    methods
        % Update the joint information
        function obj = update(obj,point,refFrame)
            obj.point = point;
            obj.refFrame = refFrame;
        end
        
        % Attach the joint to a body point
        function obj = attach(obj, varargin)
            if length(varargin) == 4
                obj.point = varargin{1};
                obj.refFrame = varargin{2};
                obj.body1 = varargin{3};
                obj.body2 = varargin{4};
            end
        end
        
        % Set the type of joint being added
        function obj = setType(obj, type, directions)
            if strcmp(type,'fixed')
                obj.constraints = '123456';
            elseif strcmp(type,'pinned')
                obj.constraints = directions; % directions refer to the refFrame set in the previous step
            end
        end
    end
end