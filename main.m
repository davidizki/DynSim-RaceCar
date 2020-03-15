clear; clc; close all;


%% I. DATA LOADING AND SET UP
tic;

% I.1 DATA TYPES DEFINITION
% e = dyn.e; >> EXTERNAL
% c = dyn.c; >> CONTROL
% g = dyn.g; >> GEOMETRY
% i = dyn.i; >> INERTIA
% s = dyn.s; >> STIFFNESS
% d = dyn.d; >> DAMPING
% p = dyn.p; >> POWERTRAIN
% t = dyn.t; >> TYRES
% a = dyn.a; >> AERODYNAMICS
% n = dyn.n; >> NUMERICS
% o = dyn.o; >> OUTPUT

% % Unfold dyn variable
% e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; t = dyn.t; a = dyn.a; n = dyn.n; o = dyn.o;
% % Fold dyn variable
% dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.t = t; dyn.a = a; dyn.n = n; dyn.o = o;

dyn = struct('e',[],'c',[],'g',[],'i',[],'s',[],'d',[],'p',[],'t',[],'a',[],'n',[],'o',[]); % main struct

% I.2 [extconditionsMap] LOAD WEATHER AND GRAVITY DATA
dyn = extconditionsMap(dyn,'Leganes','now',622); % [2020 02 29 12 00 00]

% I.3 [cardataMap] LOAD CAR DATA
dyn = cardataMap(dyn);

% I.4 LOAD SIMULATION CASE SETTINGS (track; case; parameters; simplifications; prescribed driver models)
% [track.X, track.Y, track.Z, track.Xc, track.Yc, track.Zc, track.normalsX, track.normalsY, track.normalsZ] = module_trackMap();

% I.4 [engineMap]
load('engineCurves_Adri') % 1st column: crankshaft angular velocity [rpm] | 2nd column: crankshaft torque [Nm] | 3rd column: crankshaft power [W]
ratios = [34/12 33/16 28/17 27/19 28/22 27/23]*(82/45)*(40/11);
dyn.p = struct('curves',engineCurves,'ratios',ratios,'gear',0,'rpm',0,'rpm_limit_bottom',4500,'rpm_limit_top',12500,'shift_speeds',[]);
dyn = shifter(dyn);

% figure
% title('Power Curves')
% for ii = 1:length(dyn.p.ratios)
%     plot(dyn.p.curves(:,1)*(2*pi/60)/dyn.p.ratios(ii)*dyn.g.R,dyn.p.curves(:,3)); hold on;
% end
% legend('1','2','3','4','5','6')
% 
% figure
% title('Torque Curves')
% for ii = 1:length(dyn.p.ratios)
%     plot(dyn.p.curves(:,1)*(2*pi/60)/dyn.p.ratios(ii)*dyn.g.R,dyn.p.curves(:,2)*dyn.p.ratios(ii)); hold on;
% end
% legend('1','2','3','4','5','6')

clear engineCurves ratios

% I.5 [suspensionMap]
% Suspension geometry loaded from txt coordinates files

% I.6 [tyresMap]
% Tyres reference data and fits loaded
latRefs = load('latRefs');
latModels = load('latModels');
lonRefs = load('lonRefs');
lonModels = load('lonModels');

R = (16/2)*(2.54/100); % [m]
P = 75; % kPa
camberS = 0.2; % [º]
camberD = 0; % [º]
modelData.latRefs = latRefs.toSave; modelData.latModels = latModels.fitobjects; modelData.lonRefs = lonRefs.toSave; modelData.lonModels = lonModels.fitobjects; % store ref data and coefficients

dyn.t = struct('name','FL','compound_lat',"Hoosier_1675-10_R25B_7",'compound_lon',"Hoosier_1875-10_R25B_7",'radius',R,'camber_static',camberS,'camber_dynamic',camberD,'pressure_kPa',P,'data',[]);
dyn.t(2) = struct('name','FR','compound_lat',"Hoosier_1675-10_R25B_7",'compound_lon',"Hoosier_1875-10_R25B_7",'radius',R,'camber_static',camberS,'camber_dynamic',camberD,'pressure_kPa',P,'data',[]);
dyn.t(3) = struct('name','RL','compound_lat',"Hoosier_1675-10_R25B_7",'compound_lon',"Hoosier_1875-10_R25B_7",'radius',R,'camber_static',camberS,'camber_dynamic',camberD,'pressure_kPa',P,'data',[]);
dyn.t(4) = struct('name','RR','compound_lat',"Hoosier_1675-10_R25B_7",'compound_lon',"Hoosier_1875-10_R25B_7",'radius',R,'camber_static',camberS,'camber_dynamic',camberD,'pressure_kPa',P,'data',[]);
dyn.t(5) = struct('name','ref','compound_lat',[],'compound_lon',[],'radius',[],'camber_static',[],'camber_dynamic',[],'pressure_kPa',[],'data',modelData);

clear latRefs latModels lonRefs lonModels modelData R P camberS camberD

% I.7 [aeroMap]
dyn.a.SCDmap = readmatrix('aeroMap_data2019.xlsx','Sheet','SCD','Range','B3:G7');
dyn.a.SCDf1map = readmatrix('aeroMap_data2019.xlsx','Sheet','SCDf1','Range','B3:G7');
dyn.a.SCDf2map = readmatrix('aeroMap_data2019.xlsx','Sheet','SCDf2','Range','B3:G7');

dyn.n.runtime(1) = toc;

%% II. INTEGRATION
tic;

tspan = [0 20]; % 0:0.02:10000;

% LOAD INITIAL CONDITIONS
[X0,dyn] = initialize(dyn);
opts = odeset('RelTol',1e-5,'AbsTol',1e-6,'Stats','on','refine',6); % 'refine',30 for opt. TARGET: 'RelTol',1e-6,'AbsTol',1e-8
integration_flag = true;

% KEY: dyn is not passed as variable to ode since it must be dynamically updated
[T, X] = ode15s(@(T,X) dynSimSolver_parallelV3(T,X,dyn,integration_flag), tspan, X0, opts); % ode45 or ode15s, or other stiff integrator? ode15s seems the best. Parameters that are f(t) may make the system stiff

clear tspan X0 opts
dyn.n.runtime(2)=toc;

%% III. POST-PROCESSING
% III.1 Evaluate function for variable histories
tic;
integration_flag = false;
dyn.p.rpm_history = zeros(length(T),1);
dyn.p.gear_history = zeros(length(T),1);
dyn.o.G = zeros(length(T),3);
[X0,dyn] = initialize(dyn);
[DX, dyn] = dynSimSolver_parallelV3(T,X,dyn,integration_flag); % parallel version better for integration times larger than 5000 / > 10000 T vector elements

% III.2 Plot
% 1
figure
plot(X(:,1),X(:,2))
ax = gca;
ax.YDir = 'reverse';
title('XY Trajectory')

% 2
figure
plot(T,X(:,1))
title('X Position')

% 3
figure
plot(T,X(:,7))
title('X (body) Velocity')

% 4
figure % Check error by comparing the evolution of the maximum value of x at each cycle
maxXidx = islocalmax(X(:,7));
maxXidx(1:round(length(X)/2)) = 0; % remove initial transient
plot(T(maxXidx),X(maxXidx,7))
title('Max. X Velocity at Each Cycle')

% 5
figure
title('Powertrain Data')
yyaxis left
plot(T,dyn.p.rpm);
ylim([0 14000])
yyaxis right
plot(T,dyn.p.gear)
ylim([0 7])
xlim([0 20])
legend('rpm','gear')

% 6
figure
title('Control (driver) Data')
yyaxis left
plot(T,dyn.c.theta_s);
ylim([-pi pi])
yyaxis right
plot(T,dyn.c.delta_t); hold on; % throttle
plot(T,dyn.c.delta_b); % brake
ylim([-0.05 1.2])
xlim([0 20])
legend('Steering angle','Throttle','Brake')

% 7
figure
title('Dynamics Data')
dyn.o.V = vecnorm(X(:,7:9),2,2); % row norm
% G = vecnorm(DX(:,4:6),2,2)/g; % row norm
yyaxis left
plot(T,dyn.o.V);
ylim([0 100])
yyaxis right
plot(T,dyn.o.G(:,1)); hold on; % Glon
plot(T,dyn.o.G(:,2)); % Glat
ylim([-3 3])
xlim([0 20])
legend('V','Glon','Glat')

% 8
figure
title('Tyre Forces')
plot(T,dyn.t(1).FY); hold on;
plot(T,dyn.t(2).FY); hold on;
plot(T,dyn.t(3).FY); hold on;
plot(T,dyn.t(4).FY); hold on;
legend('FL','FR','RL','RR');
grid on
xlim([0 20])

% 9
figure
title('z-Rotation')
plot(T,DX(:,4)); hold on; % DPSI
plot(T,DX(:,12)); hold on; % DR

clear i integration_flag maxXidx
dyn.n.runtime(3)=toc;
dyn.n.runtime_tot = sum(dyn.n.runtime);
