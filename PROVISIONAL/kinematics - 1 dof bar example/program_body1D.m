clc; clear; close all;

% TUTORIAL

rod = BODY1D; % create the 1D-body object
rod.geometryType = 'rod';
rod.mass = 10;
rod.xEnd1 = [0 0 0];
rod.xEnd2 = [0 2 1];
setNodalDistance(rod); % compute the length and assign it as an object property
setInertia(rod,10); % compute the inertia matrix " . For a rod, the (outer) diametre must be given
setXGC(rod); % compute the GC "
setRefFrame(rod); % "
setOrientation(rod); % "
% plotRefFrame(rod);
% plotBody1D(rod);
% plotBody1DRefFrame(rod);
setGravity(rod); % add a gravity force to the object

ground = BODY1D; % create the 1D-body object
ground.geometryType = 'ground';
ground.xEnd1 = [0 -1/2 0];
ground.xEnd2 = [0 1/2 0];
setXGC(ground); % compute the GC "
setRefFrame(ground); % "

joint1 = JOINT; % create the joint object
attach(joint1,rod.xEnd1,rod.refFrame,rod,ground); % attach the joint object to a point and two objects
setType(joint1,'pinned','12346');

rod.jointEnd1 = joint1;

% update(rod)
% update(joint,rod.xEnd1,rod.refFrame)

clear ans;

[T, R] = timeIntegration(rod,ground);

figure
for i = 1:length(T)-1
    plot([0 R(i,1)],[0 R(i,2)],'o-r','lineWidth',2,'markerSize',4);
    axis equal
    title('1 Degree Of Freedom Bar Under Gravity Field')
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    xlim([-2 2])
    ylim([-2 2])
    drawnow limitrate
    % pause((T(i+1)-T(i))/2);
end

