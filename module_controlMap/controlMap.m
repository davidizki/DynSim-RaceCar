function c = controlMap(c,T,mode)

c.delta_t = NaN(size(T)); % throttle position [0 1]
c.delta_b = NaN(size(T)); % throttle position [0 1]
c.theta_s = NaN(size(T)); % throttle position [0 1]

% Lateral Ref. Case
c.delta_t(T<5) = 0.1;
c.delta_t(T>=5) = 0;
c.delta_b(:) = 0;

c.theta_s(T<10) = 0;

TbegI = 10;
TendI = 14;
MaxAngle = pi/7;
TbegII = 16;
TendII = 20;

c.theta_s(T>=TbegI & T<TendI) = MaxAngle*(T(T>=TbegI & T<TendI)-TbegI)/(TendI-TbegI);
c.theta_s(T>=TendI) = MaxAngle;
c.theta_s(T>=TbegII & T<TendII) = MaxAngle*(TendII-T(T>=TbegII & T<TendII))/(TendII-TbegII);
c.theta_s(T>=TendII) = 0;

c.theta_s(T>=30 & T<34) = -pi/36*(T(T>=30 & T<34)-30)/4;
c.theta_s(T>=34) = -pi/36;
c.theta_s(T>=35 & T<39) = -pi/36*(39-T(T>=35 & T<39))/4;

c.theta_s(T>=39) = 0;






% c.theta_s(T>=10 & T<15) = pi/36*(T(T>=10 & T<15)-10)/4;
% c.theta_s(T>=14) = pi/36;
% c.theta_s(T>=16 & T<20) = pi/36*(20-T(T>=16 & T<20))/4;
% c.theta_s(T>=20) = 0;
% 
% c.theta_s(T>=30 & T<34) = -pi/36*(T(T>=30 & T<34)-30)/4;
% c.theta_s(T>=34) = -pi/36;
% c.theta_s(T>=35 & T<39) = -pi/36*(39-T(T>=35 & T<39))/4;
% 
% c.theta_s(T>=39) = 0;


% % Longitudinal Ref. Case
% Tfullthrottle = 5;
% c.delta_t(T<Tfullthrottle) = 0.5 + 0.5/Tfullthrottle*T(T<Tfullthrottle);
% c.delta_t(T>=Tfullthrottle) = 1;
% c.delta_b(:) = 0;
% c.theta_s(:) = 0;

end
