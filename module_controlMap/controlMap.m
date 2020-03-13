function c = controlMap(c,T,mode)

c.delta_t = NaN(size(T)); % throttle position [0 1]
c.delta_b = NaN(size(T)); % throttle position [0 1]
c.theta_s = NaN(size(T)); % throttle position [0 1]

c.delta_t(T<10) = 1;
c.delta_t(T>=10) = 0.1;

c.delta_b(:) = 0;

c.theta_s(T<10) = pi/20/10*T(T<10);
c.theta_s(T>=10) = pi/20;


end
