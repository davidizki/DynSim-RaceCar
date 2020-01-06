%% Formula UC3M lap simulator %%
%25/10/2018
% Daniel Gomez Lendinez

clear all
close all
clc
tic


circuit = 'Skid pad' ; % Posible case 'Acceleration' // 'Skid pad' // 'Getafetest2018' = not yet!!
aeroconfig = 'FSS2018'; % Posible case 'FSS2018'
engineconfig = 'FSS2018';  % Posible case 'FSS2018' // 'Aranda'
massconfig = 'FSS2018';  % Posible case 'FSS2018'
ambientconfig = 'FSS2018';  % Posible case 'FSS2018' // 'Standard' // 'Getafetest'
tireconfig = 'Normal'; % Posible case 'Normal' // 'TUE'
dynamicconfig = 'FSS2018';  % Posible case 'FSS2018'
video = 1; % 1 is True, 0 is False

disp('For set up:')
toc

%% Start configuration
tic

switch circuit
    case 'Acceleration'
        cir.x = 0:0.1:75; %Length 75 m, calculate every 10 cm seems enough
        cir.R = 1e15*ones(1,length(cir.x)); % No R
        cir.y = zeros(1,length(cir.x)); % Straight line

        
    case 'Skid pad'
        cir.alp = 0:0.02:2*pi; % Radians
        cir.R = 18.25/2*ones(1,4*length(cir.alp)); % Radius
        cir.x(1:length(cir.alp)) = cir.R(1)-cir.R(1)*cos(cir.alp); % Circ 1
        cir.y(1:length(cir.alp)) = cir.R(1)*sin(cir.alp); % Circ 1
        cir.x(length(cir.alp)+1:2*length(cir.alp)) = cir.R(1)-cir.R(1)*cos(cir.alp); % Circ 2
        cir.y(length(cir.alp)+1:2*length(cir.alp)) = cir.R(1)*sin(cir.alp); % Circ 2
        cir.x(2*length(cir.alp)+1:3*length(cir.alp)) = -cir.R(1)+cir.R(1)*cos(cir.alp); % Circ 3
        cir.y(2*length(cir.alp)+1:3*length(cir.alp)) = cir.R(1)*sin(cir.alp); % Circ 3
        cir.x(3*length(cir.alp)+1:4*length(cir.alp)) = -cir.R(1)+cir.R(1)*cos(cir.alp); % Circ 4
        cir.y(3*length(cir.alp)+1:4*length(cir.alp)) = cir.R(1)*sin(cir.alp); % Circ 4

        
    case 'Getafetest2018' % To be defined
        
    otherwise
        disp('Error: Not a valid circuit')
end

switch dynamicconfig
    case 'FSS2018'
        dyn.coglogper = 0.55;
        dyn.wheelbase = 1.58;
        dyn.wheeltrack = 1.2;
        
    otherwise
        disp('Error: Not a valid dynamic configuration')
end

switch aeroconfig
    case 'FSS2018'
        aero.Cd = 1.06;
        aero.Cl = -2.29;
        aero.A = 1.152; % in m2
        aero.front = 0.57; % Distribution, not used yet
        aero.rear = 0.43; % Distribution, not used yet 
        aero.cop = 0.45;
    otherwise
        disp('Error: Not a valid aero configuration')
end

switch engineconfig
    case 'FSS2018'
%         engine.rpm=[4086.448598 4156.542056 4310.747664 4457.943925 4591.121495 4780.373832 4920.560748 ...
%                 5053.738318 5172.897196 5292.056075 5460.280374 5635.514019 5810.747664 5950.934579 ...
%                 6126.168224 6273.364486 6469.626168 6651.869159 6869.158879 7037.383178 7205.607477 ...
%                 7338.785047 7514.018692 7738.317757 8025.700935 8207.943925 8383.17757 8614.485981 ...
%                 8789.719626 9035.046729 9301.401869 9588.785047 9904.205607 10289.71963 10521.02804 ...
%                 10843.45794 11046.72897 11285.04673 11502.33645 11733.64486];
%         engine.hp=[17.128954 19.659367 21.216545 23.552311 25.109489 26.472019 28.418491 30.170316 ...
%                 33.479319 35.425791 36.593674 38.150852 39.902676 42.238443 44.574209 47.104623 ...
%                 48.077859 50.024331 50.608273 51.581509 51.776156 53.138686 54.501217 54.695864 ...
%                 54.695864 55.474453 55.6691 56.836983 57.810219 58.978102 60.340633 61.313869 ...
%                 61.89781 62.287105 62.092457 61.703163 60.729927 60.340633 59.951338 59.367397];
%         Fenginehp = @(x) -8.346e-28*x.^8 + 5.559e-23*x.^7 + -1.586e-18*x.^6 + 2.526e-14*x.^5 ... 
%             + -2.455e-10*x.^4 + 1.488e-06*x.^3 + 0.005494*x.^2 + 11.3*x + -9934;

        engine.rpmtorq = [4072.167943 4121.13179 4177.067436 4240.017882 4331.092034 4422.166186 ...
            4506.25779 4597.364194 4702.532451 4793.671107 4842.677956 4933.752108 ...
            5031.830311 5094.813009 5136.815808 5178.786356 5241.844308 5332.950712 ...
            5445.155271 5571.400181 5725.672043 5816.767697 5914.824398 6047.998105 ...
            6153.112609 6251.244564 6363.449123 6461.667083 6580.886442 6686.129953 ... 
            6826.501221 6952.789134 7107.179252 7268.541168 7450.915235 7626.360505 ...
            7780.772124 7900.088239 8040.438005 8138.720469 8279.102486 8447.500705 ...
            8615.866671 8749.158634 8840.383294 8994.698158 9099.95242 9247.263234 ...
            9373.615651 9492.888763 9661.286981 9794.600445 9927.956912 10117.37803 ...
            10229.6901 10391.07351 10503.38558 10763.09446 10903.48723 11036.8222 ...
            11142.09796 11268.45037 11422.84049 11528.13776 11696.55748 11745.70408];


        engine.torque = [29.938594 31.20433 32.859435 34.514616 35.586159 36.657703 37.437201 38.216774 ...
            38.704529 39.192132 40.068574 41.140118 42.309061 43.672271 44.451314 45.522326 ...
            46.204273 46.983846 47.27703 47.473043 47.961329 48.838225 50.201815 51.176491 ...
            52.150863 52.833189 53.126373 53.030111 53.323371 53.129862 52.450115 52.256834 ...
            51.674562 51.48166 51.580956 50.901588 50.12467 49.542019 49.056919 48.376717 ...
            47.599647 47.212173 47.116671 47.020788 46.729804 46.828797 46.537964 46.539557 ...
            45.762335 45.568978 45.181505 44.890976 44.211153 44.018554 43.338504 42.950955 ...
            42.270905 40.81386 39.939467 39.454291 38.968812 38.19159 37.609318 36.929192 ...
            36.347072 35.958309];

        hp2w = 745.7;
        rpm2radsec = 0.104719755;
        drive.eff = 0.9;
        drive.final = 3.636;
        drive.first = 5.163;
        drive.second = 3.758;
        drive.third = 3.001;
        drive.forth = 2.589;
        drive.fifth = 2.319;
        drive.sixth = 2.139;
        wheelR = 0.1955;
        
        engine.F1 = engine.torque * drive.eff * drive.final * drive.first / wheelR;
        engine.v1 = engine.rpmtorq * rpm2radsec * wheelR / drive.first;
        engine.F2 = engine.torque * drive.eff * drive.final * drive.second / wheelR;
        engine.v2 = engine.rpmtorq * rpm2radsec * wheelR / drive.second;
        engine.F3 = engine.torque * drive.eff * drive.final * drive.third / wheelR;
        engine.v3 = engine.rpmtorq * rpm2radsec * wheelR / drive.third;
        engine.F4 = engine.torque * drive.eff * drive.final * drive.forth / wheelR;
        engine.v4 = engine.rpmtorq * rpm2radsec * wheelR / drive.forth;
        engine.F5 = engine.torque * drive.eff * drive.final * drive.fifth / wheelR;
        engine.v5 = engine.rpmtorq * rpm2radsec * wheelR / drive.fifth;
        engine.F6 = engine.torque * drive.eff * drive.final * drive.sixth / wheelR;
        engine.v6 = engine.rpmtorq * rpm2radsec * wheelR / drive.sixth;
        [val, ind] = max(engine.F1);
        engine.F1(1:ind) = val;
        aux = engine.v1(ind);
        engine.v1(1:ind) = linspace(0,aux,ind); 
        
%         for j = 1:length(engine.F1)
%             [engine.F(j),ind2(j)] = max([engine.F1(j), engine.F2(j), engine.F3(j), engine.F4(j), ...
%                 engine.F5(j), engine.F6(j)]);
% %            engine.v(j) = max([engine.v1(ind2(j)), engine.v2(ind2(j)), engine.v3(ind2(j)), ...
% %                engine.v4(ind2(j)), engine.v5(ind2(j)), engine.v6(ind2(j))]);
%         end
%         engine.v = linspace(0,max(engine.v6),length(engine.F1));
 
        engine.F=[4608 4608 4608 4608 4458 4072 3804 3141 2928 2702 2338 2158 1975 1878 1743 1682 1607 1538 1461 1301];
        engine.v=[0 10 20 26.1 29.54 34.16 40.12 46.38 50.38 56.61 62.08 70.89 75.07 80.89 86.47 90.31 93.75 99.45 103 111.9]/3.6;
        Fengine = @(x) -1.029e-06*x^8 + 0.0001148*x^7 -0.004816*x^6 + 0.08721*x^5 ...
                    -0.3947*x^4 -6.795*x^3 + 65.3*x^2 -133.5*x + 4611;

        
        
    case 'Aranda'
%         engine.F=[4000 4000 4000 4000 3984 3509 2900 2610 2247 1998 1790 1553 1165 971 737];
%         engine.v=[0 10 20 30 40.83 47.11 52.85 62.56 71.59 81.41 90.90 104.23 127.92 136.45 140]/3.6;
        Fengine = @(x)  -5.268e-07*x.^8 + 7.804e-05*x.^7  -0.004594*x.^6 + ...
                    0.1354*x.^5  -2.051*x.^4 + 14.46*x.^3  -38.42*x.^2 + 17.3*x + 4007;
    otherwise
        disp('Error: Not a valid a engine configuration')
end

switch massconfig
    case 'FSS2018'
        m.car = 262; % in Kg
        m.driver = 50; % in Kg
    otherwise
        disp('Error: Not a valid a mass configuration')
end

switch ambientconfig
    case 'FSS2018'
        g = 9.8; 
        T = 273+35; % K
        h = 20; % m
        p0 = 101325; % Pa
        p = p0 * exp(-g*h*0.0289644/288.15/8.31447);
        rho = p/(287.05*T); %in Kg/m^3

    case 'Getafetest'
        g = 9.8; 
        T = 273+10; % K
        h = 600; % m
        p0 = 101325; % Pa
        p = p0 * exp(-g*h*0.0289644/288.15/8.31447);
        rho = p/(287.05*T); %in Kg/m^3
        
    case 'standard'
        g = 9.80665; 
        T = 288.15;
        p0 = 101325;
        h = 0; % m
        p = p0 * exp(-g*h*0.0289644/288.15/8.31447);
        rho = p/(287.05*T); %in Kg/m^3
        
    otherwise
        disp('Error: Not a valid a ambient configuration')
end

switch tireconfig
    case 'Normal'
        mu = 1.4; % From Race cars
    case 'TUE'
        mu = 1.74 - 0.000128*g*m.car/4; % From TU Endhoven
    otherwise
        disp('Error: Not a valid a tire configuration')
end

disp('For configuration:')
toc 


%% Start lap timer
tic

Fdrag = @(v) 0.5*rho*(v.^2)*aero.A*aero.Cd;
Fdownforce = @(v) 0.5*rho*(v.^2)*aero.A*aero.Cl;
m.t = m.car + m.driver;
semiwheelbase = dyn.wheelbase/2;
semiwheeltrack = dyn.wheeltrack/2;

v = 0;
u = 0;
t = 0;
cir.s=0;

s = zeros(1, length(cir.x)-1);
F.yf = zeros(1, length(cir.x)-1);
F.yr = zeros(1, length(cir.x)-1);
F.y = zeros(1, length(cir.x)-1);
F.xgripf = zeros(1, length(cir.x)-1);
F.xgripr = zeros(1, length(cir.x)-1);
F.xgrip = zeros(1, length(cir.x)-1);
a  = zeros(1, length(cir.x)-1);
v_t  = zeros(1, length(cir.x)-1);
v_dt  = zeros(1, length(cir.x)-1);
 
u  = zeros(1, length(cir.x));
 
for i = 1:length(cir.x)-1
    s(i) = sqrt((cir.x(i+1)-cir.x(i)).^2 + (cir.y(i+1)-cir.y(i))^2); % length covered
    F.yf(i) = (1-dyn.coglogper)*m.t*v.^2/cir.R(i); % Centripetal  force front
    F.yr(i) = (dyn.coglogper)*m.t*v.^2/cir.R(i); % Centripetal  force rear
    F.y(i) = F.yf(i) + F.yr(i); % Centripetal  force
    F.xgripf(i) = real( sqrt( (mu*(1-dyn.coglogper)*m.t*g).^2 - F.yf(i).^2 + (1-aero.cop)*Fdownforce(v)) ); % Max x grip front
    F.xgripr(i) = real( sqrt( (mu*(dyn.coglogper)*m.t*g).^2 - F.yr(i).^2 + aero.cop*Fdownforce(v)) ); % Max x grip rear
    F.xgrip(i) = F.xgripf(i) + F.xgripr(i); % Max x grip
    a(i) = (min(F.xgrip(i), Fengine(v)) - Fdrag(v))/m.t; % Acceleration
    u(i+1) = real(sqrt(v.^2  + 2*a(i)*s(i))); % velocity
    v = u(i+1);
    dt = (u(i+1)-u(i))/a(i); 
    t = t + dt; % time
    v_t(i) = t;
    v_dt(i) = dt;
    
    
    
end

gs = a/g; % Acc in gs
alat = F.y/m.t/g; % Lateral accel

throttle = gs(gs>0);
brake = gs(gs<-0.02);

throttleperc = length(throttle) / length(gs) * 100;
brakeperc = length(brake) / length(gs) * 100;
restperc = 100 - throttleperc - brakeperc;
steeringperc = length(F.y(F.y>1)) / length(gs)*100;


switch circuit
    case 'Skid pad'
        t1 = v_t(round((length(cir.x)-1)/4));
        t2 = v_t(round((length(cir.x)-1)/2))-t1;
        t3 = v_t(round((length(cir.x)-1)*3/4))-t2-t1;
        t4 = v_t(end)-t3-t2-t1;
        t = min([t1 t2 t3 t4]);
    otherwise
        t=t;
end

disp('For lap time loop:')
toc 

%% Start ploting
tic




figure;
set(gcf, 'Position', [50, 50, 850, 650])
subplot(3,1,1)
x = cir.x;
y = cir.y;
z = zeros(size(x));
colormap(jet)
col = u*3.6;  % This is the color, vary with x in this case.
surface([x;x],[y;y],[z;z],[col;col], 'facecol','no', 'edgecol','interp','linew',2);
colorbar
title('Velocity in km/h')

subplot(3,1,2)
x = cir.x(1:end-1);
y = cir.y(1:end-1);
z = zeros(size(x));
colormap(jet)
col = gs;  % This is the color, vary with x in this case.
surface([x;x],[y;y],[z;z],[col;col], 'facecol','no', 'edgecol','interp','linew',2);
colorbar
title('Longitudinal acceleration in gs')

subplot(3,1,3)
x = cir.x(1:end-1);
y = cir.y(1:end-1);
z = zeros(size(x));
colormap(jet)
col = alat;  % This is the color, vary with x in this case.
surface([x;x],[y;y],[z;z],[col;col], 'facecol','no', 'edgecol','interp','linew',2);
colorbar
title('Lateral acceleration in gs')


filename = sprintf('%s_%s.png',circuit,date);
saveas(gcf,filename)


disp('For display:')
toc


tic 

result = sprintf('Event Time: %2.2f s',t);
disp(result)
result = sprintf('Max speed: %2.2f km/h, Mean velocity: %2.2f km/h',max(u)*3.6,mean(u)*3.6);
disp(result)
result = sprintf('Max accel: %2.2f g, Min accel: %2.2f g, Max lat accel: %2.2f g',max(gs), min(gs), max(alat));
disp(result)
result = sprintf('Time throttle: %2.2f %%, Time brake: %2.2f %%, Time rest: %2.2f %%',throttleperc, brakeperc, restperc);
disp(result)
result = sprintf('Time steering: %2.2f %%',steeringperc);
disp(result)

% menu
% 
% function menu
% % Create figure and components:
% 
% fig = uifigure;
% 
% ax = uiaxes('Parent',fig,...
%     'Position',[10 10 400 400]);
% 
% % Create a plot
% x = linspace(-2*pi,2*pi);
% y = sin(x);
% p = plot(ax,x,y);
% p.Color = 'Blue';
% 
% % Create drop-down component
% dd = uidropdown(fig,...
%     'Position',[430 210 100 22],...
%     'Items',{'Red','Yellow','Blue','Green'},...
%     'Value','Blue',...
%     'ValueChangedFcn',@(dd,event) selection(dd,p));
% end
% 
% % Create ValueChangedFcn callback:
%     function selection(dd,p)
%         val = dd.Value;
%         p.Color = val;
%     end


if video == 1

    filename = sprintf('%s_%s.avi',circuit,date);
    vid = VideoWriter (filename);
    vid.FrameRate = 1/mean(v_dt);
    open (vid);  
    
    for i = 1:length(cir.x)-1
        
        figure(999)
        plot(cir.x, cir.y, 'k:', 'LineWidth',1)
        hold on
        plot(cir.x(i), cir.y(i), 'bs', 'LineWidth',2,'MarkerSize',10, 'MarkerFaceColor',[0 0 1])
        xlim([min(cir.x)-5 max(cir.x)+5])
        ylim([min(cir.y)-5 max(cir.y)+5])
        hold off
        
        timetext = sprintf('Time: %2.3f s', v_t(i));
        veltext = sprintf('V: %3.1f km/h', u(i)*3.6);
        acctext = sprintf('Alon: %1.2f g - Alat: %1.2f g', gs(i), alat(i));
        downtext = sprintf('Downforce: %4.1f N', Fdownforce(u(i)));
               
        switch circuit
            case 'Acceleration'
            text(max(cir.x)-15, max(cir.y)+1, timetext)
            text(min(cir.x)+1, max(cir.y)+1, veltext)
            text(min(cir.x)+1, max(cir.y)+1.5, acctext)
            text(min(cir.x)+1, max(cir.y)+2, downtext)
       
         otherwise
            text(max(cir.x)-15, max(cir.y)+max(cir.y)*0.1, timetext)
            text(min(cir.x)+1, max(cir.y)+max(cir.y)*0.1, veltext)
            text(min(cir.x)+1, max(cir.y)+max(cir.y)*0.2, acctext)
            text(min(cir.x)+1, max(cir.y)+max(cir.y)*0.3, downtext)
         end
            
        drawnow;
                
        frame = getframe(gcf);
        writeVideo (vid, frame); 
              
    end
    
    close (vid)
    
end


disp('Video or data showing time:')
toc