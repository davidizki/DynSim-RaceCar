classdef FORCE < handle
    properties
        type
        magnitude
        direction
        point
    end
    
    methods
        % Set the type of force being added
        function obj = setType(obj, type)
            if strcmp(type,'gravity')
                obj.type = 'gravity';
                obj.magnitude = 9.8067;
                obj.direction = [0 0 -1];
            end
        end
        
        % Attach the force to an application point
        function obj = attach(obj, point)
            obj.point = point;
        end
    end
end