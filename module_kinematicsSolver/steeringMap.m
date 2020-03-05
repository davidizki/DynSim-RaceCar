function theta_t = steeringMap(theta_s)

% R = 0.196; % tyre radius
caster = deg2rad(5.954); % caster angle, provided in degrees
kp_offset = -0.00018; % m, kingpin offset at tyre centre
% t_mech = kp_offset + R*tan(caster); % mechanical trail

upright_arm = 0.04141*cos(caster)+kp_offset; % arm from the steering rod connecting point to the steering axis
Zrack_cfactor = 91.4/(2*pi)/1000; % m/rad, like an effective radius; ratio at the rack-pinion interface (note that ratio due to angular velocity of steering rod is neglected)
Ztaperedgb = 1/1; % ratio at the tapered gears box
% R_steeringwheel = 0.12; %steering wheel effective radius

steering_ratio = 1/upright_arm*Zrack_cfactor*Ztaperedgb;

theta_t = theta_s*steering_ratio;

end