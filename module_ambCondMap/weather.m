clear all
close all
clc

location = 'Leganes';


key = '3b335e249b442d0a31540c2a501141e1'; % Team key
options = weboptions('ContentType','json'); % Options DO NOT TOUCH
url = ['https://api.openweathermap.org/data/2.5/weather?q=', location,'&APPID=',key]; % API url
Current_Data = webread(url, options); % Need internet conection


fprintf('-----Weather report-----\n')
fprintf('Description: %s \n', Current_Data.weather.description)
fprintf('Current temp: %3.1f ºC \n', Current_Data.main.temp-273.15)
fprintf('Feeling: %3.1f ºC \n', Current_Data.main.feels_like-273.15)
fprintf('Max temp: %3.1f ºC \n', Current_Data.main.temp_max-273.15)
fprintf('Min temp: %3.1f ºC \n', Current_Data.main.temp_min-273.15)
fprintf('Pressure: %4.0f mbar \n', Current_Data.main.pressure)
fprintf('Humidity: %2.0f %% \n', Current_Data.main.humidity)
fprintf('Wind speed: %2.1f m/s \n', Current_Data.wind.speed)
fprintf('Wind dir: %3.0f º \n', Current_Data.wind.deg)