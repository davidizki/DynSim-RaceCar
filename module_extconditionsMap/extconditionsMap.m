function dyn = extconditionsMap(dyn, location, time, altitude)

% I. FUNCTIONALITY
% "conditionsMap" is used to load the external conditions data (weather, gravity)
% 
% II. INPUTS
%     dyn: core struct
%     location: place for which the weather state is desired
%     time: time at which the weather state is desired. 'now' for current conditions; whole date for future conditions.
%         Example: Y=2020;M=2;D=7;H=12;MIN=0;S=0; time = [Y M D H MIN S]; // % or directly: [2020 02 10 14 00 00]
%     altitude [optional]: altitude in meters over the sea level of the location
% 
% III. OUTPUTS
%     dyn: core struct updated with the following data
%       dyn.e.w: weather data, containing 5 different fields for the temperature (K), the pressure (Pa), the density (kgm^-3), the wind speed (ms^-1) and the wind direction (º)
%       dyn.e.g: gravity


% Unfold dyn variable
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; a = dyn.a; n = dyn.n; o = dyn.o;
e.g = [];
e.w = struct('Tamb',[],'p',[],'rho',[],'Ttrack',[],'windSpeed',[],'windDir',[]);

% 0. PARAMETERS
% location = 'Leganes';
% time = 'now';
% Y=2020;M=2;D=7;H=12;MIN=0;S=0; time = [Y M D H MIN S];
key = '3b335e249b442d0a31540c2a501141e1'; % Team key
options = weboptions('ContentType','json'); % Options DO NOT TOUCH

% 1. MAIN
if strcmp(time,'now')
%     1.1 CURRENT DATA
    url = ['https://api.openweathermap.org/data/2.5/weather?q=',location,'&APPID=',key]; % API url
    Current_Data = webread(url, options); % Needs internet conection
    lat = Current_Data.coord.lat; % latitude for gravity computation
    
    % Output
    e.w.Tamb = Current_Data.main.temp; % K
    e.w.p = Current_Data.main.pressure*100; % Pa
    R = 287.058; e.w.rho = e.w.p/R/e.w.Tamb; % kgm^-3
    if isfield(Current_Data.wind,'deg')
        e.w.windSpeed = Current_Data.wind.speed; % ms^-1
        e.w.windDir = Current_Data.wind.deg; % º
    else % wind intensity is so low that it is not possible to measure direction. Set it to zero.
        e.w.windSpeed = 0; % ms^-1
        e.w.windDir = 0; % º
    end
    
%     Ta = 32 + e.w.Tamb*9/5; % To Fahrenheit
%     Rh = Current_Data.main.humidity; % In %
%     clouds_fraction = Current_Data.clouds.all/100; % Dimensionless
%     S = 1200*clouds_fraction + 200; % Wm^-2. Assuming max of 1400 and linear with cloud fraction covering the sky
%     w = Current_Data.wind.speed;
%     Ts = 41.51 + 0.102*w + 1.71*Ta + 0.032*Rh - 0.029*S + 0.002*Ta*Rh + 5.7e-4*w*S + 0.0014*S + 4.09e-5*S^2 - 1.15e-6*Ta*S^2;
%     e.w.Ttrack = (Ts-32)*5/9; % Back to Celsius
    clouds_fraction = Current_Data.clouds.all/100;
    if e.w.Tamb > 273.15
        e.w.Ttrack = (e.w.Tamb-273.15)*(1 + 2/3*(1-clouds_fraction/100)) + 273.15; % this correlation has not been checked
    else
        e.w.Ttrack = e.w.Tamb;
    end
    
else
%     1.2 FORECAST DATA
    url = ['https://api.openweathermap.org/data/2.5/forecast?q=',location,'&APPID=',key]; % API url
    Forecast = webread(url, options); % Needs internet conection
    lat = Forecast.city.coord.lat; % latitude for gravity computation
    
    Ndata = length(Forecast.list);
    
    Time = cell(Ndata,1);
    Temp = zeros(Ndata,1);
    % Temp_min = zeros(Ndata,1);
    % Temp_max = zeros(Ndata,1);
    % Feeling = zeros(Ndata,1);
    Pressure = zeros(Ndata,1);
    % Humidity = zeros(Ndata,1);
    Ttrack = zeros(Ndata,1);
    Clouds_fraction = zeros(Ndata,1);
    Wind_speed = zeros(Ndata,1);
    Wind_deg = zeros(Ndata,1);
    
    % the weather forecast may be provided in two different formats depending on the location
    if iscell(Forecast.list) % format 1
        for i = 1:Ndata
            Time{i} = Forecast.list{i,1}.dt_txt;
            Temp(i) = Forecast.list{i,1}.main.temp;
            % Temp_min(i) = Forecast.list{i,1}.main.temp_min;
            % Temp_max(i) = Forecast.list{i,1}.main.temp_max;
            % Feeling(i) = Forecast.list{i,1}.main.feels_like;
            Pressure(i) = Forecast.list{i,1}.main.pressure;
            % Humidity(i) = Forecast.list{i,1}.main.humidity;
            Clouds_fraction(i) = Forecast.list{i,1}.clouds.all;
            if Temp(i) > 273.15
                Ttrack(i) = (Temp(i)-273.15)*(1 + 2/3*(1-Clouds_fraction(i)/100)) + 273.15; % this correlation has not been checked
            else
                Ttrack(i) = Temp(i);
            end
            if isfield(Forecast.list{i,1}.wind,'deg')
                Wind_speed(i) = Forecast.list{i,1}.wind.speed;
                Wind_deg(i) = Forecast.list{i,1}.wind.deg;
            else
                Wind_speed(i) = 0;
                Wind_deg(i) = 0;
            end
        end
    else % format 2
        for i = 1:Ndata
            Time{i} = Forecast.list(i).dt_txt;
            Temp(i) = Forecast.list(i).main.temp;
            % Temp_min(i) = Forecast.list(i).main.temp_min;
            % Temp_max(i) = Forecast.list(i).main.temp_max;
            % Feeling(i) = Forecast.list(i).main.feels_like;
            Pressure(i) = Forecast.list(i).main.pressure;
            % Humidity(i) = Forecast.list(i).main.humidity;
            Clouds_fraction(i) = Forecast.list(i).clouds.all;
            if Temp(i) > 273.15
                Ttrack(i) = (Temp(i)-273.15)*(1 + 2/3*(1-Clouds_fraction(i)/100)) + 273.15; % this correlation has not been checked
            else
                Ttrack(i) = Temp(i);
            end
            Wind_speed(i) = Forecast.list(i).wind.speed;
            if isfield(Forecast.list(i).wind,'deg')
                Wind_speed(i) = Forecast.list(i).wind.speed;
                Wind_deg(i) = Forecast.list(i).wind.deg;
            else
                Wind_speed(i) = 0;
                Wind_deg(i) = 0;
            end
        end
    end
    
    
    % Output
    Time = datetime(Time);
    Time_num = datenum(Time); % time to number
    
    Time_requested = datetime(time);
    Time_requested_num = datenum(Time_requested);
    
    done = false;
    
    if Time_num(Ndata) < Time_requested_num % out of prediction range
        error('Out of prediction range (future). Limit is %s\n', datestr(Time(Ndata)));
%         weather = 'Out of prediction range';
%         done = true;
    elseif Time_num(1) > Time_requested_num % out of prediction range
        error('Out of prediction range (past). Limit is %s\n', datestr(Time(1)));
%         weather = 'Out of prediction range';
%         done = true;
    end
    
    if done == false
        for i = 1:Ndata % if it coincides with one of the time stamps, use that data
            if Time_requested_num == Time_num(i)
                e.w.Tamb = Temp(i); % K
                e.w.p = Pressure(i)*100; % Pa
                R = 287.058; e.w.rho = e.w.p/R/e.w.Tamb; % kgm^-3
                e.w.Ttrack = Ttrack(i); % K
                e.w.windSpeed = Wind_speed(i); % ms^-1
                e.w.windDir = Wind_deg(i); % º

                done = true;
            end
        end
    end
    
    if done == false % if it does not coincide with any time stamp, interpolate
        i = 1;
        while Time_requested_num > Time_num(i)
            i = i+1;
        end
        % ith element is greater than requested; i-1th is smaller. Perform interpolation
        linterp_factor =  (Time_requested_num-Time_num(i-1))/(Time_num(i)-Time_num(i-1));
        
        e.w.Tamb = Temp(i-1) + linterp_factor*(Temp(i)-Temp(i-1)); % K
        e.w.p = (Pressure(i-1) + linterp_factor*(Pressure(i)-Pressure(i-1)))*100; % Pa
        R = 287.058; e.w.rho = e.w.p/R/e.w.Tamb; % kgm^-3
        e.w.Ttrack = Ttrack(i-1) + linterp_factor*(Ttrack(i)-Ttrack(i-1)); % K
        e.w.windSpeed = Wind_speed(i-1) + linterp_factor*(Wind_speed(i)-Wind_speed(i-1)); % ms^-1
        if ~isnan(Wind_deg)
            e.w.windDir = Wind_deg(i-1) + linterp_factor*(Wind_deg(i)-Wind_deg(i-1)); % º
        end
    end
    
%     TT = timetable(Time,Temp,Temp_min,Temp_max,Feeling,Pressure,Humidity,Wind_speed,Wind_deg);
%     fprintf('\n\n')
%     disp(TT)
end

% 2. GRAVITY ESTIMATION
if nargin == 1
  altitude = 0;
end
e.g = gravity(lat,altitude);


% Fold dyn variable
dyn.e = e;

end

% fprintf('-----Weather report-----\n')
%     fprintf('Description: %s \n', Current_Data.e.w.description)
%     fprintf('Current temp: %3.1f ºC \n', Current_Data.main.temp-273.15)
%     fprintf('Feeling: %3.1f ºC \n', Current_Data.main.feels_like-273.15)
%     fprintf('Max temp: %3.1f ºC \n', Current_Data.main.temp_max-273.15)
%     fprintf('Min temp: %3.1f ºC \n', Current_Data.main.temp_min-273.15)
%     fprintf('Pressure: %4.0f mbar \n', Current_Data.main.pressure)
%     fprintf('Humidity: %2.0f %% \n', Current_Data.main.humidity)
%     fprintf('Wind speed: %2.1f m/s \n', Current_Data.wind.speed)
%     
%     if ~isempty(Current_Data.wind.deg)
%         fprintf('Wind dir: %3.0f º \n', Current_Data.wind.deg)
%     end

% Time = datetime({Forecast.list{1,1}.dt_txt; Forecast.list{2,1}.dt_txt; Forecast.list{3,1}.dt_txt; Forecast.list{4,1}.dt_txt;...
%     Forecast.list{5,1}.dt_txt; Forecast.list{6,1}.dt_txt; Forecast.list{7,1}.dt_txt; Forecast.list{8,1}.dt_txt;...
%     Forecast.list{9,1}.dt_txt; Forecast.list{10,1}.dt_txt; Forecast.list{11,1}.dt_txt; Forecast.list{12,1}.dt_txt;...
%     Forecast.list{13,1}.dt_txt; Forecast.list{14,1}.dt_txt; Forecast.list{15,1}.dt_txt; Forecast.list{16,1}.dt_txt;...
%     Forecast.list{17,1}.dt_txt; Forecast.list{18,1}.dt_txt; Forecast.list{19,1}.dt_txt; Forecast.list{20,1}.dt_txt;
%     Forecast.list{21,1}.dt_txt; Forecast.list{22,1}.dt_txt; Forecast.list{23,1}.dt_txt; Forecast.list{24,1}.dt_txt;
%     Forecast.list{25,1}.dt_txt; Forecast.list{26,1}.dt_txt; Forecast.list{27,1}.dt_txt; Forecast.list{28,1}.dt_txt;
%     Forecast.list{29,1}.dt_txt; Forecast.list{30,1}.dt_txt; Forecast.list{31,1}.dt_txt; Forecast.list{32,1}.dt_txt;
%     Forecast.list{33,1}.dt_txt; Forecast.list{34,1}.dt_txt; Forecast.list{35,1}.dt_txt; Forecast.list{36,1}.dt_txt;
%     Forecast.list{37,1}.dt_txt; Forecast.list{38,1}.dt_txt; Forecast.list{39,1}.dt_txt; Forecast.list{40,1}.dt_txt;});
% 
% Temp = [Forecast.list{1,1}.main.temp; Forecast.list{2,1}.main.temp; Forecast.list{3,1}.main.temp; Forecast.list{4,1}.main.temp; ...
%     Forecast.list{5,1}.main.temp; Forecast.list{6,1}.main.temp; Forecast.list{7,1}.main.temp; Forecast.list{8,1}.main.temp; ...
%     Forecast.list{9,1}.main.temp; Forecast.list{10,1}.main.temp; Forecast.list{11,1}.main.temp; Forecast.list{12,1}.main.temp; ...
%     Forecast.list{13,1}.main.temp; Forecast.list{14,1}.main.temp; Forecast.list{15,1}.main.temp; Forecast.list{16,1}.main.temp; ...
%     Forecast.list{17,1}.main.temp; Forecast.list{18,1}.main.temp; Forecast.list{19,1}.main.temp; Forecast.list{20,1}.main.temp; ...
%     Forecast.list{21,1}.main.temp; Forecast.list{22,1}.main.temp; Forecast.list{23,1}.main.temp; Forecast.list{24,1}.main.temp; ...
%     Forecast.list{25,1}.main.temp; Forecast.list{26,1}.main.temp; Forecast.list{27,1}.main.temp; Forecast.list{28,1}.main.temp; ...
%     Forecast.list{29,1}.main.temp; Forecast.list{30,1}.main.temp; Forecast.list{31,1}.main.temp; Forecast.list{32,1}.main.temp; ...
%     Forecast.list{33,1}.main.temp; Forecast.list{34,1}.main.temp; Forecast.list{35,1}.main.temp; Forecast.list{36,1}.main.temp; ...
%     Forecast.list{37,1}.main.temp; Forecast.list{38,1}.main.temp; Forecast.list{39,1}.main.temp; Forecast.list{40,1}.main.temp;];
% Temp = Temp - 273.15;
% 
% Temp_min = [Forecast.list{1,1}.main.temp_min; Forecast.list{2,1}.main.temp_min; Forecast.list{3,1}.main.temp_min; Forecast.list{4,1}.main.temp_min; ...
%     Forecast.list{5,1}.main.temp_min; Forecast.list{6,1}.main.temp_min; Forecast.list{7,1}.main.temp_min; Forecast.list{8,1}.main.temp_min; ...
%     Forecast.list{9,1}.main.temp_min; Forecast.list{10,1}.main.temp_min; Forecast.list{11,1}.main.temp_min; Forecast.list{12,1}.main.temp_min; ...
%     Forecast.list{13,1}.main.temp_min; Forecast.list{14,1}.main.temp_min; Forecast.list{15,1}.main.temp_min; Forecast.list{16,1}.main.temp_min; ...
%     Forecast.list{17,1}.main.temp_min; Forecast.list{18,1}.main.temp_min; Forecast.list{19,1}.main.temp_min; Forecast.list{20,1}.main.temp_min; ...
%     Forecast.list{21,1}.main.temp_min; Forecast.list{22,1}.main.temp_min; Forecast.list{23,1}.main.temp_min; Forecast.list{24,1}.main.temp_min; ...
%     Forecast.list{25,1}.main.temp_min; Forecast.list{26,1}.main.temp_min; Forecast.list{27,1}.main.temp_min; Forecast.list{28,1}.main.temp_min; ...
%     Forecast.list{29,1}.main.temp_min; Forecast.list{30,1}.main.temp_min; Forecast.list{31,1}.main.temp_min; Forecast.list{32,1}.main.temp_min; ...
%     Forecast.list{33,1}.main.temp_min; Forecast.list{34,1}.main.temp_min; Forecast.list{35,1}.main.temp_min; Forecast.list{36,1}.main.temp_min; ...
%     Forecast.list{37,1}.main.temp_min; Forecast.list{38,1}.main.temp_min; Forecast.list{39,1}.main.temp_min; Forecast.list{40,1}.main.temp_min;];
% Temp_min = Temp_min - 273.15;
% 
% Temp_max = [Forecast.list{1,1}.main.temp_max; Forecast.list{2,1}.main.temp_max; Forecast.list{3,1}.main.temp_max; Forecast.list{4,1}.main.temp_max; ...
%     Forecast.list{5,1}.main.temp_max; Forecast.list{6,1}.main.temp_max; Forecast.list{7,1}.main.temp_max; Forecast.list{8,1}.main.temp_max; ...
%     Forecast.list{9,1}.main.temp_max; Forecast.list{10,1}.main.temp_max; Forecast.list{11,1}.main.temp_max; Forecast.list{12,1}.main.temp_max; ...
%     Forecast.list{13,1}.main.temp_max; Forecast.list{14,1}.main.temp_max; Forecast.list{15,1}.main.temp_max; Forecast.list{16,1}.main.temp_max; ...
%     Forecast.list{17,1}.main.temp_max; Forecast.list{18,1}.main.temp_max; Forecast.list{19,1}.main.temp_max; Forecast.list{20,1}.main.temp_max; ...
%     Forecast.list{21,1}.main.temp_max; Forecast.list{22,1}.main.temp_max; Forecast.list{23,1}.main.temp_max; Forecast.list{24,1}.main.temp_max; ...
%     Forecast.list{25,1}.main.temp_max; Forecast.list{26,1}.main.temp_max; Forecast.list{27,1}.main.temp_max; Forecast.list{28,1}.main.temp_max; ...
%     Forecast.list{29,1}.main.temp_max; Forecast.list{30,1}.main.temp_max; Forecast.list{31,1}.main.temp_max; Forecast.list{32,1}.main.temp_max; ...
%     Forecast.list{33,1}.main.temp_max; Forecast.list{34,1}.main.temp_max; Forecast.list{35,1}.main.temp_max; Forecast.list{36,1}.main.temp_max; ...
%     Forecast.list{37,1}.main.temp_max; Forecast.list{38,1}.main.temp_max; Forecast.list{39,1}.main.temp_max; Forecast.list{40,1}.main.temp_max;];
% Temp_max = Temp_max - 273.15;
% 
% 
% Feeling = [Forecast.list{1,1}.main.feels_like; Forecast.list{2,1}.main.feels_like; Forecast.list{3,1}.main.feels_like; Forecast.list{4,1}.main.feels_like; ...
%     Forecast.list{5,1}.main.feels_like; Forecast.list{6,1}.main.feels_like; Forecast.list{7,1}.main.feels_like; Forecast.list{8,1}.main.feels_like; ...
%     Forecast.list{9,1}.main.feels_like; Forecast.list{10,1}.main.feels_like; Forecast.list{11,1}.main.feels_like; Forecast.list{12,1}.main.feels_like; ...
%     Forecast.list{13,1}.main.feels_like; Forecast.list{14,1}.main.feels_like; Forecast.list{15,1}.main.feels_like; Forecast.list{16,1}.main.feels_like; ...
%     Forecast.list{17,1}.main.feels_like; Forecast.list{18,1}.main.feels_like; Forecast.list{19,1}.main.feels_like; Forecast.list{20,1}.main.feels_like; ...
%     Forecast.list{21,1}.main.feels_like; Forecast.list{22,1}.main.feels_like; Forecast.list{23,1}.main.feels_like; Forecast.list{24,1}.main.feels_like; ...
%     Forecast.list{25,1}.main.feels_like; Forecast.list{26,1}.main.feels_like; Forecast.list{27,1}.main.feels_like; Forecast.list{28,1}.main.feels_like; ...
%     Forecast.list{29,1}.main.feels_like; Forecast.list{30,1}.main.feels_like; Forecast.list{31,1}.main.feels_like; Forecast.list{32,1}.main.feels_like; ...
%     Forecast.list{33,1}.main.feels_like; Forecast.list{34,1}.main.feels_like; Forecast.list{35,1}.main.feels_like; Forecast.list{36,1}.main.feels_like; ...
%     Forecast.list{37,1}.main.feels_like; Forecast.list{38,1}.main.feels_like; Forecast.list{39,1}.main.feels_like; Forecast.list{40,1}.main.feels_like;];
% Feeling = Feeling- 273.15;
% 
% Pressure = [Forecast.list{1,1}.main.pressure; Forecast.list{2,1}.main.pressure; Forecast.list{3,1}.main.pressure; Forecast.list{4,1}.main.pressure; ...
%     Forecast.list{5,1}.main.pressure; Forecast.list{6,1}.main.pressure; Forecast.list{7,1}.main.pressure; Forecast.list{8,1}.main.pressure; ...
%     Forecast.list{9,1}.main.pressure; Forecast.list{10,1}.main.pressure; Forecast.list{11,1}.main.pressure; Forecast.list{12,1}.main.pressure; ...
%     Forecast.list{13,1}.main.pressure; Forecast.list{14,1}.main.pressure; Forecast.list{15,1}.main.pressure; Forecast.list{16,1}.main.pressure; ...
%     Forecast.list{17,1}.main.pressure; Forecast.list{18,1}.main.pressure; Forecast.list{19,1}.main.pressure; Forecast.list{20,1}.main.pressure; ...
%     Forecast.list{21,1}.main.pressure; Forecast.list{22,1}.main.pressure; Forecast.list{23,1}.main.pressure; Forecast.list{24,1}.main.pressure; ...
%     Forecast.list{25,1}.main.pressure; Forecast.list{26,1}.main.pressure; Forecast.list{27,1}.main.pressure; Forecast.list{28,1}.main.pressure; ...
%     Forecast.list{29,1}.main.pressure; Forecast.list{30,1}.main.pressure; Forecast.list{31,1}.main.pressure; Forecast.list{32,1}.main.pressure; ...
%     Forecast.list{33,1}.main.pressure; Forecast.list{34,1}.main.pressure; Forecast.list{35,1}.main.pressure; Forecast.list{36,1}.main.pressure; ...
%     Forecast.list{37,1}.main.pressure; Forecast.list{38,1}.main.pressure; Forecast.list{39,1}.main.pressure; Forecast.list{40,1}.main.pressure;];
% 
% Humidity = [Forecast.list{1,1}.main.humidity; Forecast.list{2,1}.main.humidity; Forecast.list{3,1}.main.humidity; Forecast.list{4,1}.main.humidity; ...
%     Forecast.list{5,1}.main.humidity; Forecast.list{6,1}.main.humidity; Forecast.list{7,1}.main.humidity; Forecast.list{8,1}.main.humidity; ...
%     Forecast.list{9,1}.main.humidity; Forecast.list{10,1}.main.humidity; Forecast.list{11,1}.main.humidity; Forecast.list{12,1}.main.humidity; ...
%     Forecast.list{13,1}.main.humidity; Forecast.list{14,1}.main.humidity; Forecast.list{15,1}.main.humidity; Forecast.list{16,1}.main.humidity; ...
%     Forecast.list{17,1}.main.humidity; Forecast.list{18,1}.main.humidity; Forecast.list{19,1}.main.humidity; Forecast.list{20,1}.main.humidity; ...
%     Forecast.list{21,1}.main.humidity; Forecast.list{22,1}.main.humidity; Forecast.list{23,1}.main.humidity; Forecast.list{24,1}.main.humidity; ...
%     Forecast.list{25,1}.main.humidity; Forecast.list{26,1}.main.humidity; Forecast.list{27,1}.main.humidity; Forecast.list{28,1}.main.humidity; ...
%     Forecast.list{29,1}.main.humidity; Forecast.list{30,1}.main.humidity; Forecast.list{31,1}.main.humidity; Forecast.list{32,1}.main.humidity; ...
%     Forecast.list{33,1}.main.humidity; Forecast.list{34,1}.main.humidity; Forecast.list{35,1}.main.humidity; Forecast.list{36,1}.main.humidity; ...
%     Forecast.list{37,1}.main.humidity; Forecast.list{38,1}.main.humidity; Forecast.list{39,1}.main.humidity; Forecast.list{40,1}.main.humidity;];
% 
% 
% Wind_speed = [Forecast.list{1,1}.wind.speed; Forecast.list{2,1}.wind.speed; Forecast.list{3,1}.wind.speed; Forecast.list{4,1}.wind.speed; ...
%     Forecast.list{5,1}.wind.speed; Forecast.list{6,1}.wind.speed; Forecast.list{7,1}.wind.speed; Forecast.list{8,1}.wind.speed; ...
%     Forecast.list{9,1}.wind.speed; Forecast.list{10,1}.wind.speed; Forecast.list{11,1}.wind.speed; Forecast.list{12,1}.wind.speed; ...
%     Forecast.list{13,1}.wind.speed; Forecast.list{14,1}.wind.speed; Forecast.list{15,1}.wind.speed; Forecast.list{16,1}.wind.speed; ...
%     Forecast.list{17,1}.wind.speed; Forecast.list{18,1}.wind.speed; Forecast.list{19,1}.wind.speed; Forecast.list{20,1}.wind.speed; ...
%     Forecast.list{21,1}.wind.speed; Forecast.list{22,1}.wind.speed; Forecast.list{23,1}.wind.speed; Forecast.list{24,1}.wind.speed; ...
%     Forecast.list{25,1}.wind.speed; Forecast.list{26,1}.wind.speed; Forecast.list{27,1}.wind.speed; Forecast.list{28,1}.wind.speed; ...
%     Forecast.list{29,1}.wind.speed; Forecast.list{30,1}.wind.speed; Forecast.list{31,1}.wind.speed; Forecast.list{32,1}.wind.speed; ...
%     Forecast.list{33,1}.wind.speed; Forecast.list{34,1}.wind.speed; Forecast.list{35,1}.wind.speed; Forecast.list{36,1}.wind.speed; ...
%     Forecast.list{37,1}.wind.speed; Forecast.list{38,1}.wind.speed; Forecast.list{39,1}.wind.speed; Forecast.list{40,1}.wind.speed;];
% 
% 
% Wind_deg = [Forecast.list{1,1}.wind.deg; Forecast.list{2,1}.wind.deg; Forecast.list{3,1}.wind.deg; Forecast.list{4,1}.wind.deg; ...
%     Forecast.list{5,1}.wind.deg; Forecast.list{6,1}.wind.deg; Forecast.list{7,1}.wind.deg; Forecast.list{8,1}.wind.deg; ...
%     Forecast.list{9,1}.wind.deg; Forecast.list{10,1}.wind.deg; Forecast.list{11,1}.wind.deg; Forecast.list{12,1}.wind.deg; ...
%     Forecast.list{13,1}.wind.deg; Forecast.list{14,1}.wind.deg; Forecast.list{15,1}.wind.deg; Forecast.list{16,1}.wind.deg; ...
%     Forecast.list{17,1}.wind.deg; Forecast.list{18,1}.wind.deg; Forecast.list{19,1}.wind.deg; Forecast.list{20,1}.wind.deg; ...
%     Forecast.list{21,1}.wind.deg; Forecast.list{22,1}.wind.deg; Forecast.list{23,1}.wind.deg; Forecast.list{24,1}.wind.deg; ...
%     Forecast.list{25,1}.wind.deg; Forecast.list{26,1}.wind.deg; Forecast.list{27,1}.wind.deg; Forecast.list{28,1}.wind.deg; ...
%     Forecast.list{29,1}.wind.deg; Forecast.list{30,1}.wind.deg; Forecast.list{31,1}.wind.deg; Forecast.list{32,1}.wind.deg; ...
%     Forecast.list{33,1}.wind.deg; Forecast.list{34,1}.wind.deg; Forecast.list{35,1}.wind.deg; Forecast.list{36,1}.wind.deg; ...
%     Forecast.list{37,1}.wind.deg; Forecast.list{38,1}.wind.deg; Forecast.list{39,1}.wind.deg; Forecast.list{40,1}.wind.deg;];
% 
% TT = timetable(Time,Temp,Temp_min,Temp_max,Feeling,Pressure,Humidity,Wind_speed,Wind_deg);
% 
% fprintf('--------------------------------------------------\n')
% 
% disp(TT)



