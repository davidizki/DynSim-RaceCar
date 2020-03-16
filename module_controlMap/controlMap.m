function c = controlMap(c,T,mode)

c.delta_t = NaN(size(T)); % throttle position [0 1]
c.delta_b = NaN(size(T)); % throttle position [0 1]
c.theta_s = NaN(size(T)); % throttle position [0 1]

% c.delta_t(T<10) = 1;
% c.delta_t(T>=10) = 0;

c.theta_s(T<10) = 0;

c.theta_s(T>=10 & T<15) = pi/36*(T(T>=10 & T<15)-10)/4;
c.theta_s(T>=14) = pi/36;
c.theta_s(T>=16 & T<20) = pi/36*(20-T(T>=16 & T<20))/4;
c.theta_s(T>=20) = 0;

c.theta_s(T>=30 & T<34) = -pi/36*(T(T>=30 & T<34)-30)/4;
c.theta_s(T>=34) = -pi/36;
c.theta_s(T>=35 & T<39) = -pi/36*(39-T(T>=35 & T<39))/4;
c.theta_s(T>=39) = 0;

c.delta_t(:) = 0.15;
c.delta_b(:) = 0;
% c.theta_s(:) = 0;

end
