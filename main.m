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

global nnzeros
nnzeros = 0;

% I.2 [extconditionsMap] LOAD WEATHER AND GRAVITY DATA
dyn = extconditionsMap(dyn,'Leganes','now',622); % [2020 02 29 12 00 00]



% I.4 LOAD SIMULATION CASE SETTINGS (track; case; parameters; simplifications; prescribed driver models)
% [track.X, track.Y, track.Z, track.Xc, track.Yc, track.Zc, track.normalsX, track.normalsY, track.normalsZ] = module_trackMap();

% I.5 [suspensionMap]
% Suspension geometry loaded from txt coordinates files

% I.6 [tyresMap]
% Tyres reference data and fits loaded
latRefsFY = load('module_tyresMap/lat/latRefsFY');
latModelsFY = load('module_tyresMap/lat/latModelsFY');
latRefsMX = load('module_tyresMap/lat/latRefsMX');
latModelsMX = load('module_tyresMap/lat/latModelsMX');
latRefsMZ = load('module_tyresMap/lat/latRefsMZ');
latModelsMZ = load('module_tyresMap/lat/latModelsMZ');
lonRefsFX = load('module_tyresMap/lon/lonRefsFX');
lonModelsFX = load('module_tyresMap/lon/lonModelsFX');
% lonRefsMZ = load('module_tyresMap/lon/lonRefsMZ');
% lonModelsMZ = load('module_tyresMap/lon/lonModelsMZ');

% SETUP
P = 75; % kPa
camberS = 0.2; % [�]
camberD = 0; % [�]

% store ref data and coefficients
modelData.latRefsFY = latRefsFY.toSave; modelData.latModelsFY = latModelsFY.fitobjects;
modelData.latRefsMX = latRefsMX.toSave; modelData.latModelsMX = latModelsMX.fitobjects;
modelData.latRefsMZ = latRefsMZ.toSave; modelData.latModelsMZ = latModelsMZ.fitobjects;
modelData.lonRefsFX = lonRefsFX.toSave; modelData.lonModelsFX = lonModelsFX.fitobjects;
% modelData.lonRefsMZ = lonRefsMZ.toSave; modelData.lonModelsMZ = lonModelsMZ.fitobjects;

dyn.t = struct('name','FL','compound_lat',"Hoosier_1675-10_R25B_7",'compound_lon',"Hoosier_1875-10_R25B_7",'camber_static',camberS,'camber_dynamic',camberD,'pressure_kPa',P,'data',[]);
dyn.t(2) = struct('name','FR','compound_lat',"Hoosier_1675-10_R25B_7",'compound_lon',"Hoosier_1875-10_R25B_7",'camber_static',camberS,'camber_dynamic',camberD,'pressure_kPa',P,'data',[]);
dyn.t(3) = struct('name','RL','compound_lat',"Hoosier_1675-10_R25B_7",'compound_lon',"Hoosier_1875-10_R25B_7",'camber_static',camberS,'camber_dynamic',camberD,'pressure_kPa',P,'data',[]);
dyn.t(4) = struct('name','RR','compound_lat',"Hoosier_1675-10_R25B_7",'compound_lon',"Hoosier_1875-10_R25B_7",'camber_static',camberS,'camber_dynamic',camberD,'pressure_kPa',P,'data',[]);
dyn.t(5) = struct('name','ref','compound_lat',[],'compound_lon',[],'camber_static',[],'camber_dynamic',[],'pressure_kPa',[],'data',modelData);

load('module_tyresMap/tyreStiffnessData');
for ii = 1:4
    dyn.t(ii).Rstatic = tyreVertical([tyreVertical.name]==dyn.t(ii).compound_lat).R;
    dyn.t(ii).RLfun = tyreVertical([tyreVertical.name]==dyn.t(ii).compound_lat).RLfun;
    dyn.t(ii).Kt = tyreVertical([tyreVertical.name]==dyn.t(ii).compound_lat).Kt;
end

clear latRefsFY latModelsFY latRefsMX latModelsMX latRefsMZ latModelsMZ lonRefsFX lonModelsFX lonRefsMZ lonModelsMZ modelData R P camberS camberD tyreVertical

% I.3 [cardataMap] LOAD CAR DATA
dyn = cardataMap(dyn);

% I.4 [engineMap]
load('engineCurves_Adri.mat','engineCurves') % 1st column: crankshaft angular velocity [rpm] | 2nd column: crankshaft torque [Nm] | 3rd column: crankshaft power [W]
load('fuelMap_formula_3.11.18.mat','fuelMap')
fuelMap.Z = reshape(fuelMap.Z,length(fuelMap.delta_t_real),length(fuelMap.rpm));
ratios = [34/12 33/16 28/17 27/19 28/22 27/23]*(82/45)*(40/11);
dyn.p = struct('curves',engineCurves,'ratios',ratios,'gear',0,'rpm',0,'rpm_limit_bottom',0,'rpm_limit_top',12500,'rpm_idle',3500,'omega_limit_top',...
    convangvel(12500,'rpm','rad/s'),'omega_idle',convangvel(3500,'rpm','rad/s'),'shift_speeds',[],'fuelMap',fuelMap);
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
% 
% figure
% title('Fuel Map')
% [X,Y] = ndgrid(fuelMap.delta_t_real,fuelMap.rpm);
% Z = fuelMap.Z;
% surf(X,Y,Z); hold on;
% ax = gca;
% ax.YDir = 'reverse';
% 
% F = griddedInterpolant(X,Y,Z,"linear");
% Xq = 55; Yq = 8200; Zq = F(Xq,Yq);
% plot3(Xq,Yq,Zq,'*r','markerSize',5);

% Engine properties
dyn = engineProps(dyn);

clear engineCurves ratios fuelMap X Y Z Xq Yq Zq

% I.7 [aeroMap]
dyn.a.SCDmap = readmatrix('aeroMap_data2019.xlsx','Sheet','SCD','Range','B3:G7');
dyn.a.SCDf1map = readmatrix('aeroMap_data2019.xlsx','Sheet','SCDf1','Range','B3:G7');
dyn.a.SCDf2map = readmatrix('aeroMap_data2019.xlsx','Sheet','SCDf2','Range','B3:G7');

dyn.n.runtime(1) = toc;

%% II. INTEGRATION
tic;

tspan = [0 50]; % 0:0.02:10000;
Tmaxplot = tspan(2);

% LOAD INITIAL CONDITIONS
[X0,dyn] = initialize(dyn);
opts = odeset('RelTol',1e-5,'AbsTol',1e-6,'Stats','on','refine',6); % 'refine',30 for opt. TARGET: 'RelTol',1e-6,'AbsTol',1e-8
global integration_flag
integration_flag = true;

% KEY: dyn is not passed as variable to ode since it must be dynamically updated
[T, X] = ode15s(@(T,X) dynSimSolver_parallelV4(T,X,dyn), tspan, X0, opts); % ode45 or ode15s, or other stiff integrator? ode15s seems the best. Parameters that are f(t) may make the system stiff

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
[DX,dyn] = dynSimSolver_parallelV4(T,X,dyn); % parallel version better for integration times larger than 5000 / > 10000 T vector elements

% III.2 Plot
% 1
figure
plot(X(:,1),X(:,2))
ax = gca;
ax.YDir = 'reverse';
title('XY Trajectory')
xlabel('X Position [m]')
ylabel('Y Position [m]')
axis equal
grid on

% 2
figure
plot(T,X(:,1))
title('X Position')
xlabel('Time [s]')
ylabel('X Position [m]')
grid on

% 3
figure
plot(T,X(:,7))
title('X (body) Velocity')
xlabel('Time [s]')
ylabel('U [m/s]')
grid on

% 4
figure % Check error by comparing the evolution of the maximum value of x at each cycle
maxXidx = islocalmax(X(:,7));
maxXidx(1:round(length(X)/2)) = 0; % remove initial transient
plot(T(maxXidx),X(maxXidx,7))
title('Max. X Velocity at Each Cycle - Accuracy Check')
xlabel('Time [s]')
ylabel('Max X Position per Cycle [m]')
grid on

% 5
figure
title('Powertrain Data')
yyaxis left
plot(T,dyn.p.rpm);
ylim([0 14000])
yyaxis right
plot(T,dyn.p.gear)
ylim([0 7])
xlim([0 Tmaxplot])
legend('Angular Velocity [rpm]','Gear [#]')
xlabel('Time [s]')
grid on

% 6
figure
title('Controls (Driver) Data')
yyaxis left
plot(T,dyn.c.theta_s);
ylim([-pi pi])
yyaxis right
plot(T,dyn.c.delta_t); hold on; % throttle
plot(T,dyn.c.delta_b); % brake
plot(T,dyn.c.delta_c); % clutch
ylim([-0.05 1.2])
xlim([0 Tmaxplot])
legend('Steering Angle [-]','Throttle [-]','Brake [-]','Clutch [-]')
xlabel('Time [s]')
grid on

% 7
figure
title('Dynamics Data')
% dyn.o.V = vecnorm(X(:,7:9),2,2); % row norm
% G = vecnorm(DX(:,4:6),2,2)/g; % row norm
yyaxis left
plot(T,dyn.o.speed);
ylim([0 100])
yyaxis right
plot(T,dyn.o.G(:,1)); hold on; % Glon
plot(T,dyn.o.G(:,2)); % Glat
ylim([-3 3])
xlim([0 Tmaxplot])
legend('V [m/s]','G_{lon} [-]','G_{lat} [-]')
xlabel('Time [s]')
grid on

% 8
figure
title('Tyre Forces')

subplot(3,1,1);
title('FX')
plot(T,dyn.t(1).FX(:,1)); hold on;
plot(T,dyn.t(2).FX(:,1)); hold on;
plot(T,dyn.t(3).FX(:,1)); hold on;
plot(T,dyn.t(4).FX(:,1)); hold on;
legend('FL [N]','FR [N]','RL [N]','RR [N]');
xlim([0 Tmaxplot])
xlabel('Time [s]')
grid on

subplot(3,1,2);
title('FY')
plot(T,dyn.t(1).FY(:,2)); hold on;
plot(T,dyn.t(2).FY(:,2)); hold on;
plot(T,dyn.t(3).FY(:,2)); hold on;
plot(T,dyn.t(4).FY(:,2)); hold on;
legend('FL [N]','FR [N]','RL [N]','RR [N]');
xlim([0 Tmaxplot])
xlabel('Time [s]')
grid on

subplot(3,1,3);
title('FZ')
plot(T,dyn.t(1).Fnormal(:,3)); hold on;
plot(T,dyn.t(2).Fnormal(:,3)); hold on;
plot(T,dyn.t(3).Fnormal(:,3)); hold on;
plot(T,dyn.t(4).Fnormal(:,3)); hold on;
legend('FL [N]','FR [N]','RL [N]','RR [N]');
xlim([0 Tmaxplot])
xlabel('Time [s]')
grid on

% 9
figure
title('Z-Rotation (Relevant Data)')
plot(T,DX(:,4)); hold on; % DPSI
plot(T,DX(:,12)); hold on; % DR
xlabel('Time [s]')
legend('DPSI [s^{-1}]','DR [s^{-2}]');
grid on

% 10
figure
title('Degrees of Freedom (6x2)')
% plot(T,X(:,1)); hold on;
% plot(T,X(:,2)); hold on;
plot(T,X(:,3)); hold on;
plot(T,X(:,4)); hold on;
plot(T,X(:,5)); hold on;
plot(T,X(:,6)); hold on;
% plot(T,X(:,7),'--'); hold on;
plot(T,X(:,8),'--'); hold on;
plot(T,X(:,9),'--'); hold on;
plot(T,X(:,10),'--'); hold on;
plot(T,X(:,11),'--'); hold on;
plot(T,X(:,12),'--'); hold on;
xlabel('Time [s]') % next line: 'Xe [m]','Ye [m]',    'U [m/s]',
legend('Ze [m]','PSI [-]','THETA [-]','PHI [-]','V [m/s]','W [m/s]','P [s^{-1}]','Q [s^{-1}]','R [s^{-1}]');
grid on

% 11
figure
title('Derivatives of Degrees of Freedom (6x2)')
% plot(T,DX(:,1)); hold on;
% plot(T,DX(:,2)); hold on;
plot(T,DX(:,3)); hold on;
plot(T,DX(:,4)); hold on;
plot(T,DX(:,5)); hold on;
plot(T,DX(:,6)); hold on;
% plot(T,DX(:,7),'--'); hold on;
plot(T,DX(:,8),'--'); hold on;
plot(T,DX(:,9),'--'); hold on;
plot(T,DX(:,10),'--'); hold on;
plot(T,DX(:,11),'--'); hold on;
plot(T,DX(:,12),'--'); hold on;
xlabel('Time [s]') % next line: 'DXe [m/s]','DYe [m/s]',     'DU [m/s^2]',
legend('DZe [m/s]','DPSI [s^{-1}]','DTHETA [s^{-1}]','DPHI [s^{-1}]','DV [m/s^2]','DW [m/s^2]','DP [s^{-2}]','DQ [s^{-2}]','DR [s^{-2}]');
grid on

% 12
figure
title('Moments acting on the centre of gravity')
plot(T,dyn.o.M.total(:,1)); hold on;
plot(T,dyn.o.M.total(:,2)); hold on;
plot(T,dyn.o.M.aero(:,2)); hold on;
plot(T,dyn.o.M.total(:,3)); hold on;
xlabel('Time [s]')
legend('M_x','M_y','M_y_{,aero}','M_z');
grid on

% % 13
% figure
% step = 100;
% translations = X(1:step:end,1:3);
% rotations = eul2quat(X(1:step:end,4:6),"ZYX");
% plotTransforms(translations,rotations,'FrameSize',3.25,'MeshFilePath','module_postProcessing/C3-R19_AEROMAP_Default.STL','Parent',gca,'MeshColor',"black");
% ax = gca;
% ax.YDir = ax.ZDir;
% ax.YDir = 'reverse';
% ax.ZDir = 'reverse';
% title('XYZ Trajectory and Rotations')
% xlabel('X Position [m]')
% ylabel('Y Position [m]')
% axis equal
% grid on
% zlim([-10 10])

% 14
figure()
title('Heights')
subplot(2,1,1)
plot(T,dyn.o.corner(1).r(:,3)*1e3); hold on;
plot(T,dyn.o.corner(2).r(:,3)*1e3); hold on;
xlabel('Time [s]')
ylabel('z [mm]')
ax = gca;
ax.YDir = 'reverse';
legend('h_{FL}','h_{FR}');
grid on
subplot(2,1,2)
plot(T,dyn.o.corner(3).r(:,3)*1e3); hold on;
plot(T,dyn.o.corner(4).r(:,3)*1e3); hold on;
xlabel('Time [s]')
ylabel('z [mm]')
ax = gca;
ax.YDir = 'reverse';
legend('h_{RL}','h_{RR}');
grid on

clear i integration_flag maxXidx Tmaxplot rotations translations step ax
dyn.n.runtime(3)=toc;
dyn.n.runtime_tot = sum(dyn.n.runtime);

rollGradient = max(abs(rad2deg(X(:,6))))/max(abs(dyn.o.G(:,2)))


% % 8 - old code (V3)
% figure
% title('Tyre Forces')
% 
% subplot(3,1,1);
% title('FX')
% plot(T,dyn.t(1).FX); hold on;
% plot(T,dyn.t(2).FX); hold on;
% plot(T,dyn.t(3).FX); hold on;
% plot(T,dyn.t(4).FX); hold on;
% legend('FL [N]','FR [N]','RL [N]','RR [N]');
% xlim([0 Tmaxplot])
% xlabel('Time [s]')
% grid on
% 
% subplot(3,1,2);
% title('FY')
% plot(T,dyn.t(1).FY); hold on;
% plot(T,dyn.t(2).FY); hold on;
% plot(T,dyn.t(3).FY); hold on;
% plot(T,dyn.t(4).FY); hold on;
% legend('FL [N]','FR [N]','RL [N]','RR [N]');
% xlim([0 Tmaxplot])
% xlabel('Time [s]')
% grid on
% 
% subplot(3,1,3);
% title('FZ')
% plot(T,dyn.t(1).FZ); hold on;
% plot(T,dyn.t(2).FZ); hold on;
% plot(T,dyn.t(3).FZ); hold on;
% plot(T,dyn.t(4).FZ); hold on;
% legend('FL [N]','FR [N]','RL [N]','RR [N]');
% xlim([0 Tmaxplot])
% xlabel('Time [s]')
% grid on

% Moments old (V3)
% figure()
% plot(T,dyn.o.MX.total)
% hold on
% plot(T,dyn.o.MX.total2)
% 
% figure()
% plot(T,dyn.o.MY.total)
% hold on
% plot(T,dyn.o.MY.total2)
% 
% figure()
% plot(T,dyn.o.MZ.total)
% hold on
% plot(T,dyn.o.MZ.total2)

figure
yyaxis left
plot(T,convangvel(X(:,13),'rad/s','rpm')); hold on;
yyaxis right
plot(T,convangvel(X(:,14),'rad/s','rpm')); hold on;
legend('engine','rear wheels');