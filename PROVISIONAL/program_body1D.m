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
plotRefFrame(rod);
plotBody1D(rod);
plotBody1DRefFrame(rod);
setGravity(rod); % add a gravity force to the object

ground = BODY1D; % create the 1D-body object
ground.geometryType = 'ground';
ground.xEnd1 = [0 -1/2 0];
ground.xEnd2 = [0 1/2 0];

joint1 = JOINT; % create the joint object
attach(joint1,rod.xEnd1,rod.refFrame,rod,ground); % attach the joint object to a point and two objects
setType(joint1,'pinned','12346');

rod.jointEnd1 = joint1;

% update(rod)
% update(joint,rod.xEnd1,rod.refFrame)

clear ans;


