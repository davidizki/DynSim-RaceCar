function FXengine = engineMap(engine,speed,delta_t,rho)

P = interp1(engine.curves(:,1),engine.curves(:,3),engine.rpm).*delta_t; % Assuming linear with throttle
FXengine = P./speed;

end