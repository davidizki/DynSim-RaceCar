clear; clc; close all;

% DATA TYPES DEFINITION



% LOAD CAR DATA
% Suspension geometry loaded from txt coordinates files

% LOAD MAPS
[track.X, track.Y, track.Z, track.Xc, track.Yc, track.Zc, track.normalsX, track.normalsY, track.normalsZ] = module_trackMap();

% LOAD INITIAL CONDITIONS

% LOAD SIMULATION CASE SETTINGS (track; case; parameters; simplifications)


% -------------- BIGGER LOOP BEGINS

% APPLY DRIVER INPUTS (prescribed driver models)

% GET DATA FROM MAPS (using data from ICs / previous step)

% PROJECT ALL COMPUTED FORCES INTO THE BODY-FIXED FRAME
% Use: https://es.mathworks.com/help/robotics/ref/eul2rotm.html
% FRAMES:
%     -EARTH FIXED
%     -BODY FIXED (for the tensor of inertia not to change with body motion...)
%     -AERO (wind)
%     -TRACK FIXED: for the forces at the tyres contact patches
%     -FRAMES FOR THE SUSPENSION POINTS AND TYRES

% POSE THE SET OF EQUATIONS FOR THE BODY (including wheel rotation effects; wind) -> Coupled with the suspension and tyres motion?

% POSE THE SUSPENSION POINTS (and tyres) EQUATIONS OF MOTION

% INTEGRATE 1 TIME STEP

% -------------- BIGGER LOOP ENDS

%% POINT-MASS MODEL

tspan = 1:100;
X0 = [0; 0; 0; 10; 0; 0];
opts = odeset('RelTol',1e-16,'AbsTol',1e-16);
[T, X] = ode45(@(T,X) pointMass(T,X), tspan, X0,opts);

figure
plot(X(:,1),X(:,2))

figure
plot(T,X(:,1))
