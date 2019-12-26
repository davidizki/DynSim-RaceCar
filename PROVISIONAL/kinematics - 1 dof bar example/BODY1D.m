classdef BODY1D < handle
    properties
        geometryType
        mass
        nodalDistance
        inertia
        axialStiffness
        xGC % row vector
        refFrame
        orientation
        xEnd1 % row vector
        xEnd2 % row vector
        jointEnd1
        jointEnd2
        loads
        torsorGC
    end
    
% 1. update
% 2. setNodalDistance
% 3. setInertia
% 4. setXGC
% 5. setRefFrame
% 6. setOrientation
% 7. setGravity

% plotBody1D
% plotRefFrame
% plotBody1DRefFrame
    methods
        function obj = update(obj)
            setXGC(obj)
            setRefFrame(obj)
            setOrientation(obj)
            setGravity(obj)
        end
        
        function obj = setNodalDistance(obj)
            obj.nodalDistance = norm(obj.xEnd1 - obj.xEnd2);
        end
        
        function obj = setInertia(obj, varargin)
            if length(varargin) == 2
                innerDiam = min(cell2mat(varargin));
                outerDiam = max(cell2mat(varargin));
            elseif length(varargin) == 1
                diam = cell2mat(varargin);
            end
            
            if isempty(obj.geometryType)
                disp('Geometry type has not been defined yet. Inertia matrix cannot be computed.');
            elseif isempty(obj.mass)
                disp('Mass property has not been defined yet. Inertia matrix cannot be computed.');
            elseif isempty(obj.nodalDistance)
                disp('Nodal distance property has not been defined yet. Inertia matrix cannot be computed.');
            else
                if strcmp(obj.geometryType,'rod')
                    temp = diam^2;
                    Iz = 1/2*obj.mass*temp;
                    Ir = 1/12*obj.mass*(3*temp + obj.nodalDistance^2);
                    
                    obj.inertia = diag([Iz Ir Ir]);
                    
                elseif strcmp(obj.geometryType,'tube')
                    temp = (innerDiam/2)^2 + (outerDiam/2)^2;
                    Iz = 1/2*obj.mass*temp;
                    Ir = 1/12*obj.mass*(3*temp + obj.nodalDistance^2);
                    
                    obj.inertia = diag([Iz Ir Ir]);
                    
                else
                    disp('Geometry Type not valid. Assign "rod" or "tube",')
                end
            end
        end
        
        function obj = setXGC(obj)
            obj.xGC = mean([obj.xEnd1; obj.xEnd2]);
        end
        
        function obj = setRefFrame(obj)
            % x-axis goes along the 1D body, from end point 1 to end point 2.
            % y-axis defined such that it has initially no z-component.
            % z-axis defined such that it has initially positive z-component (but this is imposed in y definition: z is just the cross prod. of x and y)
            
            x = (obj.xEnd2-obj.xEnd1)/norm(obj.xEnd2-obj.xEnd1);
            
            if (x(1) && x(2)) == 0
                y(1) = 1;
                y(2) = 0;
            elseif x(1) == 0
                y(2) = 0;
                y(1) = 1;
            elseif x(2) == 0
                y(1) = 0;
                y(0) = 1;
            else
                y(2) = ((x(2)/x(1))^2+1)^(-1/2);
                y(1) = -x(2)/x(1)*y(2);
            end
                
            if x(1)*y(2) - x(2)*y(1) < 0
                y(2) = -y(2); y(1) = -y(1);
            else
            end
            y(3) = 0;
            
            z = cross(x,y);
            
            obj.refFrame(:,1) = x; % column vectors
            obj.refFrame(:,2) = y;
            obj.refFrame(:,3) = z;
        end
        
        function obj = setOrientation(obj)
            % xb = obj.refFrame(:,1);
            % yb = obj.refFrame(:,2);
            % zb = obj.refFrame(:,3);
            % x = [1 0 0]; y = [0 1 0]; z = [0 0 1];
            % obj.orientation = [atan2(norm(cross(x,xb)),dot(x,xb)) atan2(norm(cross(y,yb)),dot(y,yb)) atan2(norm(cross(z,zb)),dot(z,zb))];
            
            rotm = obj.refFrame;
            obj.orientation = rotm2eul(rotm);
        end
        
        function obj = setGravity(obj)
            flag = 0;
            if ~isempty(obj.loads) % if there is at least one load already defined
                for i = length(obj.loads) % check if gravity has already been defined
                    if strcmp(obj.loads{i}.type,'gravity')
                        flag=i;
                        break
                    end
                end
            end
            if flag == 0 % if not yet defined, create it
                obj.loads{end+1} = FORCE;
                setType(obj.loads{end},'gravity'); % now it is end, if not we are keeping increasing the size fo the array!
                attach(obj.loads{end},obj.xGC);
            else % if already defined, just upload the point of application
                attach(obj.loads{flag},obj.xGC);
            end
        end
        
%         function obj = setReactions(obj)
%             if ~isempty(obj.jointEnd1)
%                 % CALCULAR LAS REACCIONES EN LAS DIRECCIONES CONSTRAINT DE LA JOINT
%             end
%             
%             if ~isempty(obj.jointEnd2)
%                 
%             end
%         end
        
%         function obj = setTorsorGC(obj,joints)
%             % Tranforms the loads into a torsor placed at the GC
%             % We are solving the motion of just 1 body, so we just need as input the body and its associated joints. Each end an associated joint?
%             % torsor is a [1x6] row vector: [FX, FY, FZ, MX, MY, MZ];
%             
%             for i = 1:length(obj.loads)
%                 
%             end
%             
%             for joints
%                 
%                 if ground... -> Get RXNs
%                         
%                 end
%                 
%                 obj.torsorGC = [];
%                 
%             end
%         end
        
        % Plotting utilities
        function obj = plotBody1D(obj)
            figure
            plot3([obj.xEnd1(1) obj.xEnd2(1)],[obj.xEnd1(2) obj.xEnd2(2)],[obj.xEnd1(3) obj.xEnd2(3)],'k','lineWidth',2);
        end
        
        function obj = plotRefFrame(obj)
            figure
            hold on; plot3([0 obj.refFrame(1,1)],[0 obj.refFrame(2,1)],[0 obj.refFrame(3,1)],'r'); % x unit vector
            hold on; plot3([0 obj.refFrame(1,2)],[0 obj.refFrame(2,2)],[0 obj.refFrame(3,2)],'b'); % y unit vector
            hold on; plot3([0 obj.refFrame(1,3)],[0 obj.refFrame(2,3)],[0 obj.refFrame(3,3)],'g'); % z unit vector
        end
        
        function obj = plotBody1DRefFrame(obj)
            figure
            hold on; plot3([obj.xEnd1(1) obj.xEnd2(1)],[obj.xEnd1(2) obj.xEnd2(2)],[obj.xEnd1(3) obj.xEnd2(3)],'k','lineWidth',2);
            hold on; plot3(obj.xGC(1)+[0 obj.refFrame(1,1)],obj.xGC(2)+[0 obj.refFrame(2,1)],obj.xGC(3)+[0 obj.refFrame(3,1)],'r');
            hold on; plot3(obj.xGC(1)+[0 obj.refFrame(1,2)],obj.xGC(2)+[0 obj.refFrame(2,2)],obj.xGC(3)+[0 obj.refFrame(3,2)],'b');
            hold on; plot3(obj.xGC(1)+[0 obj.refFrame(1,3)],obj.xGC(2)+[0 obj.refFrame(2,3)],obj.xGC(3)+[0 obj.refFrame(3,3)],'g');
        end

        
        %
        %             function setJoint
        %
        %             end
    end
end