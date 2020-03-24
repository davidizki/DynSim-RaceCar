clear; clc; close all

cd('module_postProcessing');

% AEROMAP GENERATION CODE - BY DAVID IZQUIERDO, FORMULA UC3M
% The objective of this code is to generate the car geometries in order to perform CFD simulations in different attitudes of the car. As an input, several
% geometric parameters of the car, parameters for the aeromap and a folder containing each of the car parts in STL format are given. As an output, the
% geometries specified by the aeromap parameters are obtained.

% WARNINGS
% -Car parts must be provided to the code by placing them in the folder "carParts"
% -Providing the files in any other format will lead to a crash of the program
% -Some naming conventions must be respected. Particularly, the files associated to each of the wheels must be named 'wheelFR.STL' for front left and so on.
% -The name of the output files can be easily changed in line ~405
% -Some of the aeromap parameters are set to zero because the associated functionalities are still to be implemented

% MAIN LEGEND
% p = pitch; r = roll; y = yaw; s = steer; v = velocity;

% CODE SECTIONS
% 0. PARAMETERS: reference data about the car is given by the user
% 1. MAP INPUTS: reference data for the generation of the aeromap operating points is given by the user
% 2. MAP CONSTRUCTION: from the data in the preivous sections, the aeromap matrix data is built in the variable "map". THIS IS THE MAIN VARIABLE of the code.
% 3. STLs CREATION: from the aeromap data matrix, the car geometries are created from the individual parts STLs

%% 0. PARAMETERS (struct. "p")
% 0.1 General Car Data (esp.) for Rotation
p.wheelRadius = 194; % [mm] == wheel radius
p.wb = 1580; % [mm] wheelbase
p.hGC = 300; % [mm] ~GC height
p.tr = 177.8/2*2 + 511.1*2; % [mm] lateral distance between wheel centers.
p.trf = p.tr; % [mm] The same track in both axes
p.trr = p.tr; % [mm] 
% floorWidth = 1402; % [mm] maximum width of the monocoque at floor height - NOT USED (track used instead)

p.hFrontAxle = p.wheelRadius; % [mm]

p.steerRATIO = 100/32; % [-] angle steered by steering wheel / angle steered by wheel

% 0.2 Default Car State == Static
default.rhf = 37.5; % [mm] nominal height of the monocoque at floor height
default.rhr = default.rhf;
default.rhbase = (default.rhf + default.rhr)./2;
default.pitch = rad2deg(atan((default.rhf'-default.rhr')./p.wb)); % = 0
default.rhlat = default.rhf;
default.roll = rad2deg(atan((default.rhlat-default.rhbase)./(p.tr/2))); % = 0
default.yaw = 0; % [º]
default.steer = 0; % [º]
default.w = 0; % [pseudoflag]
default.drs = 0; % [flag]
default.v = 60; % [km/h]

% 0.3 Limits and Parameters for Some Map Variables Range Definition
p.steerLIM = 100; % [º] maximum angle that can be steered by the steering wheel

p.carLength = 700 + 1/2*p.wheelRadius + p.wb + 1/2*p.wheelRadius + 250; % [mm] in terms of max. in regulations

p.vMIN = 20; % [km/h]
p.vMAX = 140; % [km/h]

%% 1. INPUTS - MAP LIMITS & SENSITIVITIES. "data" struct. (I) Input; (D) Derived.

% (I) 1a. Ride Height - Front
range.rhfMIN = 27.5; % [mm]
range.rhfSTEP = 5;
range.rhfMAX = 47.5;
data.rhf = range.rhfMIN:range.rhfSTEP:range.rhfMAX;

% (I) 1b. Ride Height - Rear -> Same range of values swept for front and rear rhs
range.rhrMIN = range.rhfMIN; % [mm]
range.rhrSTEP = range.rhfSTEP;
range.rhrMAX = range.rhfMAX;
data.rhr = range.rhrMIN:range.rhrSTEP:range.rhrMAX;

% create the "mesh"
[data.rhf, data.rhr] = meshgrid(data.rhf,data.rhr); % create the "mesh" of rhf and rhr

% Delete the configurations which are coincident with the default one (for
% the other variables is easier, since there are no combined cases
indx1 = find(~(data.rhf-default.rhf)); % find indices of data.rhf that match default
indx2 = find(~(data.rhr-default.rhr));
del = intersect(indx1,indx2); % get the indices in which BOTH are the same as in default
for i = 1:length(del)
    data.rhf(del(i)) = []; % delete the coincident values; shorten the vector
    data.rhr(del(i)) = [];
end

% (D) 1c. Base Ride Height
data.rhbase = 0; % NOT DEFINED HERE, justinitializing field -> Just a function of rhf, rhr

% (D) 1d. Pitch Angle
data.pitch = 0; % NOT DEFINED HERE, justinitializing field -> Just a function of rhf, rhr

% (I) 2a. Ride Height - Lateral (refers to left side: from =0 to >0 roll)
range.rhlatMIN = 37.5; % [mm]
range.rhlatSTEP = 5;
range.rhlatMAX = 57.5;
data.rhlat = range.rhlatMIN:range.rhlatSTEP:range.rhlatMAX;
data.rhlat(data.rhlat==default.rhlat) = [];

% (D) 2b. Roll Angle
data.roll = 0; % NOT DEFINED HERE -> Just a function of rhlat

% (I) 3. Yaw Angle
range.yawMIN = 0; % [º]
range.yawSTEP = 3;
range.yawMAX = 30;
data.yaw = range.yawMIN:range.yawSTEP:range.yawMAX;
data.yaw(data.yaw==default.yaw) = [];

% (I) 4. Steer Angle
range.steerMIN = 0; % [º]
range.steerSTEP = p.steerLIM/4;
range.steerMAX = p.steerLIM;
data.steer = range.steerMIN:range.steerSTEP:range.steerMAX;
data.steer(data.steer==default.steer) = [];

% (I) 5. Wake (defined in terms of distance wrt. car in front)
range.wMIN = 500; % [mm]
range.wSTEP = p.carLength;
range.wMAX = range.wMIN + 3*range.wSTEP;
data.w = range.wMIN:range.wSTEP:range.wMAX;
data.w(data.w==default.w) = [];

% (I) 6. DRS
data.drs(1) = 0; % [flag] OFF
data.drs(2) = 1; % ON
data.drs(data.drs==default.drs) = [];

% (I) 7. Velocity (Reynolds) Effects
range.vMIN = p.vMIN; % [km/h]
range.vSTEP = 40;
range.vMAX = p.vMAX;
data.v = range.vMIN:range.vSTEP:range.vMAX;
data.v(data.v==default.v) = [];

% (I) 8. Operating Points (combine previous points. To have more accurate
% data at those poitns and to validate superposition principle)

% VALUES TO BE DEFINED YET!!!!
% COLUMN VECTORS!!!
OP.definition = {'Acceleration','Braking','TurnLSpeed','TurnMSpeed','TurnHSpeed'};
OP.rhf = 37.5*ones(numel(OP.definition),1); % [mm] nominal height of the monocoque at floor height
OP.rhr = OP.rhf;
OP.rhbase = (OP.rhf + OP.rhr)./2;
OP.pitch = rad2deg(atan((OP.rhf-OP.rhr)./p.wb)); % = 0
OP.rhlat = OP.rhf;
OP.roll = rad2deg(atan((OP.rhlat-OP.rhbase)./(p.tr/2))); % = 0
OP.yaw = 0*ones(numel(OP.definition),1); % [º]
OP.steer = 0*ones(numel(OP.definition),1); % [º]
OP.w = 0*ones(numel(OP.definition),1); % [pseudoflag]
OP.drs = 0*ones(numel(OP.definition),1); % [flag]
OP.v = 60*ones(numel(OP.definition),1); % [km/h]

%% 2. MAP CONSTRUCTION. "map" struct. to be passed to the main function.

%Nconfig = default (is 1) + hlong + hlat + yaw + steer + w + drs + v
Nconfig = numel(default) + numel(data.rhf) + numel(data.rhlat) + numel(data.yaw) + numel(data.steer) + numel(data.w) + numel(data.drs) + numel(data.v) + numel(OP.definition); 

% Define one by one the columns of the "map" table.
fields = fieldnames(data); % cell array with fields is stored

map = zeros(Nconfig,numel(fields)); % map size is Nconfig x Nfields
NbeforeCONST = 1; % constant1 size: previous size + previous field size
NafterCONST = Nconfig - (NbeforeCONST + numel(data.(fields{1})) + numel(OP.definition)); % constant2 size: total - (constant1 size + current field size - OP size)
configs = cell(1,Nconfig); % name of each configuration
for i = 1:numel(fields)
    switch fields{i}
        case 'rhbase'
            data.(fields{i}) = (data.rhf + data.rhr)./2;
        case 'pitch' % i=4
            data.(fields{i}) = rad2deg(atan((data.rhf-data.rhr)./p.wb));
        case 'roll' % i=6
            data.(fields{i}) = rad2deg(atan((data.rhlat-mean([default.rhf default.rhr]))./(p.tr/2))); % default rhf-rhr because here pitch and roll do not change simultaneaously
        otherwise
            data.(fields{i}) = data.(fields{i})(:); % == reshape(X,[numel(X),1]) -> make the fields column vectors
    end
    
    if (i > 1) && ~(strcmp(fields{i},'rhr')) && ~(strcmp(fields{i},'rhbase')) && ~(strcmp(fields{i},'pitch')) && ~(strcmp(fields{i},'roll'))
                                  % Note that fields "rhr", "pitch" and
                                  % "roll" do not add extra configs (just restate of redefine existing ones)
        NbeforeCONST = NbeforeCONST + numel(data.(fields{i-1})); 
        NafterCONST = NafterCONST - numel(data.(fields{i}));
    end
    
    map(:,i) = [default.(fields{i})*ones(NbeforeCONST,1); data.(fields{i}); default.(fields{i})*ones(NafterCONST,1); OP.(fields{i})]; % const1 + variable (study of that field) + const2 + OP
    
    for j = 1:numel(data.(fields{i}))
        configs{NbeforeCONST + j} = strcat(fields{i}(1),num2str(j));
    end
end

% Name the remaining rows: default and OPs
configs{1} = 'Default';
for j = 1:numel(OP.definition)
    configs{Nconfig-numel(OP.definition)+j} = OP.definition{j};
end

mainVars = numel(fields);

% BUILD TABLE FROM MATRIX; RENAME TABLE COLS, NAME TABLE ROWS
fields{end+1} = 'Road_VZ';
fields{end+1} = 'Road_VX';
fields{end+1} = 'AngularV';
fields{end+1} = 'Front_Rot_AxisX';
fields{end+1} = 'Front_Rot_AxisY';
fields{end+1} = 'Front_Rot_AxisZ';
fields{end+1} = 'FrontR_Rot_PointX';
fields{end+1} = 'FrontR_Rot_PointY';
fields{end+1} = 'FrontR_Rot_PointZ';
fields{end+1} = 'FrontL_Rot_PointX';
fields{end+1} = 'FrontL_Rot_PointY';
fields{end+1} = 'FrontL_Rot_PointZ';
fields{end+1} = 'Rear_Rot_AxisX';
fields{end+1} = 'Rear_Rot_AxisY';
fields{end+1} = 'Rear_Rot_AxisZ';
fields{end+1} = 'Rear_Rot_PointX'; % Both wheels in the rear axle remain always in the same axis of rotation
fields{end+1} = 'Rear_Rot_PointY';
fields{end+1} = 'Rear_Rot_PointZ';
fields{end+1} = 'CD';
fields{end+1} = 'CDf1';
fields{end+1} = 'CDf2';
fields{end+1} = 'xcp';
fields{end+1} = 'Omega';
fields{end+1} = 'kappa';
map = [map zeros(numel(configs),numel(fields)-mainVars)];

map = array2table(map,'VariableNames',fields,'RowNames',configs);

save('map.mat','map');

clearvars indx1 indx2 indx del i j fields NbeforeCONST NafterCONST mainVars

%% 3. STLs CREATION

% User inputs
version = 'AEROMAP'; % Version of the car. For the file name
inputFOLDER = 'carParts';
outputFOLDER = 'allGeometries';

% 
addpath(fullfile(pwd,'stlTools')) % add path to auxiliary functions
addpath(fullfile(pwd,inputFOLDER)) % add path to input

if exist(outputFOLDER, 'dir')
    rmdir(outputFOLDER, 's') % delete it -recursively- if it already exists
end
mkdir(outputFOLDER) % create output folder

files = dir(fullfile(pwd,inputFOLDER)); % get part files
N = numel(files);

for config = 1:1 % 1:size(map,1)
    
    % 0. Create temporary merging folder
    tempFOLDER = 'MERGE_FOLDER';
    if ~exist(tempFOLDER, 'dir')
        mkdir(tempFOLDER) % create it if it does not exist
    else
        cd(tempFOLDER) % remove its content if it does exist
        delete *
        cd ..
    end
    
    for i = 3:N % from 3 due to 2 "ghost" files
        fnm = files(i).name;

        % 1. IMPORT
        [v, f, n, name] = stlReadAscii(fnm);

        %%%%% Everything defined as STANDARD: X-RIGHT; Y-FRONT; Z-UP; .STL is corrected to be adapted to this system %%%%%
        % 2. INPUTS FROM THE MAPPING
        origin = convlength([0 -p.wb/2 -p.wheelRadius+default.rhbase],'m','km'); % [0 0 0] is the center of the front axis
                                                            % The origin is the middle of the wb, at the default rhbase height
        pitch = deg2rad(map.pitch(config));
        roll = deg2rad(map.roll(config));
        yaw = deg2rad(map.yaw(config));
        angles = -[pitch roll yaw]; % around x, y and z; - to meet + convention
        
        steer = deg2rad(map.steer(config)/p.steerRATIO);
        heaving = convlength(-default.rhbase + map.rhbase(config),'m','km'); % Equivalent to the actual 'mm' 'm'

        % 2.1 .STL COORDINATES REDEFINITION (if already ok, comment this section)
        % ***Redefinition of the coordinates system due to axes: x=-x; y=z; z=y;
        %%%%% POSITIVE SIGN CONVENTION:
        % + pitch when front goes up
        % + roll when right sinks
        % + yaw when it points to the left
        v_corrected = [-v(:,1) v(:,3) v(:,2)];

        % origin = [-origin(1) origin(3) origin(2)]; % Easier to change the .STL to the standard system than vice versa
        % angles = [-angle(1) angle(3) angle(2)];
        
        % road velocity
        map.Road_VZ(config) = -convvel(map.v(config)*cos(yaw),'km/h','m/s');
        map.Road_VX(config) = -convvel(map.v(config)*sin(yaw),'km/h','m/s');
        
        % angular velocity of the tyres
        map.AngularV(config) = -convvel(map.v(config),'km/h','m/s')/p.hFrontAxle;
        
        
        % 3. ROTATION
        if 0 % contains(fnm,'wheel') % wheels
            angles = [0 0 0];
            
            % default ROTaxis
            ROTaxis = [1 0 0];
            ROTaxis_corrected = [-ROTaxis(1) ROTaxis(3) ROTaxis(2)];
            
            %%%%% STEER the front wheels in position
            switch fnm
                case 'wheelFR.STL'
                    angles = -[0 0 steer];
                    origin = convlength([p.trf/2 0 0],'m','km'); % [0 0 0] is the center of the front axis
                case 'wheelFL.STL'
                    angles = -[0 0 steer];
                    origin = convlength([-p.trf/2 0 0],'m','km'); % [0 0 0] is the center of the front axis
            end
            
            % for ROTpoint
            switch fnm
                case 'wheelFR.STL'
                    ROTpoint_corrected = origin; % origin is given directly in the "correct" coords for the rotation
                case 'wheelFL.STL'
                    ROTpoint_corrected = origin;
                case 'wheelRR.STL'
                    ROTpoint = [0 0 -p.wb/1000];
                    ROTpoint_corrected = [-ROTpoint(1) ROTpoint(3) ROTpoint(2)];
                case 'wheelRL.STL'
                    ROTpoint = [0 0 -p.wb/1000];
                    ROTpoint_corrected = [-ROTpoint(1) ROTpoint(3) ROTpoint(2)];
            end
            
            rotMatrix = rotationVectorToMatrix(angles); % Rodrigues formulation
            v_input = v_corrected - repmat(origin,length(v),1); % Substract the origin coords.
            v_output = rotMatrix*v_input'; % Rotate
            v_write1 = v_output' + repmat(origin,length(v),1); % Add again the origin coords.
            
            %%%%%% YAW: Wheels are not moved except for steer cases and yaw
            angles = -[0 0 yaw];
            origin = convlength([0 -p.wb/2 -p.wheelRadius+default.rhbase],'m','km');
            rotMatrix = rotationVectorToMatrix(angles); % Rodrigues formulation
            
            v_input = v_write1 - repmat(origin,length(v),1); % Substract the origin coords.
            v_output = rotMatrix*v_input'; % Rotate
            v_write = v_output' + repmat(origin,length(v),1); % Add again the origin coords.
            
            switch fnm
                case 'wheelFR.STL'
                    anglesforROTaxis = -[0 0 yaw + steer];
                case 'wheelFL.STL'
                    anglesforROTaxis = -[0 0 yaw + steer];
                case 'wheelRR.STL'
                    anglesforROTaxis = -[0 0 yaw];
                case 'wheelRL.STL'
                    anglesforROTaxis = -[0 0 yaw];
            end
            rotMatrix = rotationVectorToMatrix(anglesforROTaxis);
            ROTaxis_output = rotMatrix*ROTaxis_corrected';
            ROTaxis_write = [-ROTaxis_output(1) ROTaxis_output(3) ROTaxis_output(2)];
            
            anglesforROTpoint = -[0 0 yaw];
            rotMatrix = rotationVectorToMatrix(anglesforROTpoint);
            ROTpoint_output = rotMatrix*(ROTpoint_corrected-origin)' + origin';
            ROTpoint_write = [-ROTpoint_output(1) ROTpoint_output(3) ROTpoint_output(2)];
            
            % Save the data for the rotation in the CFD setup in the "map"
            switch fnm % STEER the front wheels in position
                case 'wheelFR.STL'
                    map.Front_Rot_AxisX(config) = ROTaxis_write(1);
                    map.Front_Rot_AxisY(config) = ROTaxis_write(2);
                    map.Front_Rot_AxisZ(config) = ROTaxis_write(3);
                    map.FrontR_Rot_PointX(config) = ROTpoint_write(1);
                    map.FrontR_Rot_PointY(config) = ROTpoint_write(2);
                    map.FrontR_Rot_PointZ(config) = ROTpoint_write(3);
                case 'wheelFL.STL'
                    map.FrontL_Rot_PointX(config) = ROTpoint_write(1);
                    map.FrontL_Rot_PointY(config) = ROTpoint_write(2);
                    map.FrontL_Rot_PointZ(config) = ROTpoint_write(3);
                case 'wheelRR.STL'
                    map.Rear_Rot_AxisX(config) = ROTaxis_write(1);
                    map.Rear_Rot_AxisY(config) = ROTaxis_write(2);
                    map.Rear_Rot_AxisZ(config) = ROTaxis_write(3);
                    map.Rear_Rot_PointX(config) = ROTpoint_write(1);
                    map.Rear_Rot_PointY(config) = ROTpoint_write(2);
                    map.Rear_Rot_PointZ(config) = ROTpoint_write(3);
            end
            
        else % whichever part that is not a wheel
            
            angles = [pi/2 0 pi/2];
            rotMatrix = eul2rotm(angles,'XYZ'); % Rodrigues formulation
            
            v_input = v_corrected - repmat(origin,length(v),1); % Substract the origin coords.
            v_output = rotMatrix*v_input'; % Rotate
            v_write = v_output' + repmat(origin,length(v),1); % Add again the origin coords.

            % 4. TRANSLATION
            v_write(:,3) = v_write(:,3) + heaving;
        end


        
        % 5. EXPORT (1 PART/LOOP)
        v_writecorrected = [-v_write(:,1) v_write(:,3) v_write(:,2)]; % ***Undo the change of coordinate system
        
        cd(tempFOLDER)
        stlWrite(fnm,f,v_writecorrected,'mode','ascii','Title',fnm(1:end-4)) % end-4 to delete .STL ending
        cd ..

    end
    
    % 5. CREATE CONFIG. MODEL (1 MODEL/LOOP)
    cd(tempFOLDER)
    fid = fopen('merger.ps1', 'wt' );
    outputNAME = strcat('C3-R19_',version,'_',configs{config},'.STL');
    fprintf(fid, strcat('Get-Content .\\*.stl | Add-Content .\\',outputNAME)); % fprintf(fid, '%f,%f,%f,%f\n', a1, a2, a3, a4);
    fclose(fid); % double \\ is used because it is a special character and otherwise MATLAB does not interpret it right
    system('powershell -inputformat none -file merger.ps1');
    % MOVE CAR STL TO OUTPUT FOLDER
    movefile(outputNAME,strcat('../',outputFOLDER)); % move the assembly to the output folder
    cd ..
    rmdir(tempFOLDER, 's') % delete temporary merging folder and all its files
    
    fprintf('Geometry %d / %d exported\n', config, size(map,1))
end

clearvars angles ans config f fid fnm heaving i n N name Nconfig origin pitch roll rollMatrix yaw

cd('..');

% DEVELOPMENT TASKS
% 1. Fix parts name problem -> OK
% 2. Fix rotation problem -> OK
% 3. Tyres: don't move - EXCEPT FOR YAW -> OK
% 3.b ADD WHEEL STEERING -> OK
% 4. Delete rotated part files so that only full car remains -> OK
% 5. Change name of the OUTPUT folder -> OK
% 6. Change name of the car: add VERSION -> OK
% 7. Include the loop of the maps; add the name of the map (i.e. height
% +10) to the car name when exporting -> OK
% 8. Implement front and rear height (input: default heights, wheelbase,
% desired heights -> use roll to get them -> OK
