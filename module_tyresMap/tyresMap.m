function [FY, MX, MZ, FX, SA, SL] = tyresMap(FZ,V,lambda_muy,t,models,alpha,sigma)

% t is a scalar element of an array of structs containing the information for one tyre

% Reference data in tyres input data is in the following units:
% -P: kPa
% -IA: º
% -V: km/h
P = t.pressure_kPa;
IA = abs(t.camber_static + t.camber_dynamic); % CHECK SIGN: only >0 values from experimental data. Assume that >0 ~= <0 (symmetric)
V = 3.6*V; % from m/s to km/h

%% LATERAL
% 1. Evaluate tyre properties
compound_lat = t.compound_lat;

% 2. Get weights of interpolation
iTyre = find(strcmp({models.data.latRefsFY(:).name},compound_lat));
latRefs = models.data.latRefsFY(iTyre); % get the reference (lat) data for the desired compound

% IA
if sum(IA == latRefs.ref.IA) == 1 % if exactly matches one of the references, use exaclty that data
    refIA_down = find(IA == latRefs.ref.IA);
    refIA_up = "off";
else
    for ii = 1:length(latRefs.ref.IA)
        if ii == length(latRefs.ref.IA) % if it is greater than the biggest IA tested, use the biggest IA
            refIA_down = ii;
            refIA_up = "off";
            break
        end
        if latRefs.ref.IA(ii+1) > IA % if lies somewhere in between, interpolate solutions
            refIA_down = ii;
            refIA_up = ii+1;
            % linear interpolation: DOWN*x_1 + UP*(1-x_1) = IA; x_1 = (UP-IA)/(UP-DOWN)
            refIA_downWeight = (latRefs.ref.IA(refIA_up)-IA)/(latRefs.ref.IA(refIA_up)-latRefs.ref.IA(refIA_down));
            refIA_upWeight = 1-refIA_downWeight;
            break
        end
        
    end
end

% P
if sum(P == latRefs.ref.P) == 1 % if exactly matches one of the references, use exaclty that data
    refP_down = find(P == latRefs.ref.P);
    refP_up = "off";
else
    for ii = 1:length(latRefs.ref.P)
        if ii == 1
            if latRefs.ref.P(ii) > P % if it is smaller than the smallest P tested, use the smallest P
                refP_down = ii;
                refP_up = "off";
                break
            end
        end
        if ii == length(latRefs.ref.P) % if it is greater than the biggest P tested, use the biggest P
            refP_down = ii;
            refP_up = "off";
            break
        end
        if latRefs.ref.P(ii+1) > P % if lies somewhere in between, interpolate solutions
            refP_down = ii;
            refP_up = ii+1;
            % linear interpolation: DOWN*x_1 + UP*(1-x_1) = P; x_1 = (UP-P)/(UP-DOWN)
            refP_downWeight = (latRefs.ref.P(refP_up)-P)/(latRefs.ref.P(refP_up)-latRefs.ref.P(refP_down));
            refP_upWeight = 1-refP_downWeight;
            break
        end
        
    end
end

% % V
% if sum(V == latRefs.ref.V) == 1 % if exactly matches one of the references, use exaclty that data
%     refV_down = find(V == latRefs.ref.V);
%     refV_up = "off";
% else
%     for ii = 3:length(latRefs.ref.V) % starting from 3 since 1,2 values for V are extremely small -no data available-
%         if ii == 3
%             if latRefs.ref.V(ii) > V % if it is smaller than the smallest V tested, use the smallest V
%                 refV_down = ii;
%                 refV_up = "off";
%                 break
%             end
%         end
%         if ii == length(latRefs.ref.V) % if it is greater than the biggest V tested, use the biggest V
%             refV_down = ii;
%             refV_up = "off";
%             break
%         end
%         if latRefs.ref.V(ii+1) > V % if lies somewhere in between, interpolate solutions
%             refV_down = ii;
%             refV_up = ii+1;
%             % linear interpolation: DOWN*x_1 + UP*(1-x_1) = V; x_1 = (UP-V)/(UP-DOWN)
%             refV_downWeight = (latRefs.ref.V(refV_up)-V)/(latRefs.ref.V(refV_up)-latRefs.ref.V(refV_down));
%             refV_upWeight = 1-refV_downWeight;
%             break
%         end
%         
%     end
% end

% next 3 lines to check if interps are ok (only valid if values lying between two reference data points)
% latRefs.ref.IA(refIA_down)*refIA_downWeight+latRefs.ref.IA(refIA_up)*refIA_upWeight
% latRefs.ref.P(refP_down)*refP_downWeight+latRefs.ref.P(refP_up)*refP_upWeight
% latRefs.ref.V(refV_down)*refV_downWeight+latRefs.ref.V(refV_up)*refV_upWeight


% 1st Interpolation - V
% Note that there are only 3 values for the velocity, and only the central one has been tested with different values of IA, p
% First identify the lower and upper velocity sfits
% NOT IMPLEMENTED YET: little data available, not a great effect in accuracy, may slow down the code


% 2nd Interpolation - P
iV = 4;
sfitsFY = models.data.latModelsFY(iTyre,iV,:,:); % remove compound and velocity (the first two indexes must be == 1 for sfits
sfitsMX = models.data.latModelsMX(iTyre,iV,:,:);
sfitsMZ = models.data.latModelsMZ(iTyre,iV,:,:);

controlListPacejkaFY = models.data.latRefsFY.controlListCoeffs;
controlListPacejkaMX = models.data.latRefsMX.controlListCoeffs;
controlListPacejkaMZ = models.data.latRefsMZ.controlListCoeffs;

% alpha = -0.3:0.01:0.3;
FZ0 = latRefs.ref.FZ(5);

if ~strcmp(refP_up,"off")
     FYdown = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsFY{1,1,refP_down,refIA_down}),controlListPacejkaFY);
     FYup = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsFY{1,1,refP_up,refIA_down}),controlListPacejkaFY);
     FY = FYdown*refP_downWeight + FYup*refP_upWeight;
     
     MXdown = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsMX{1,1,refP_down,refIA_down}),controlListPacejkaMX);
     MXup = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsMX{1,1,refP_up,refIA_down}),controlListPacejkaMX);
     MX = MXdown*refP_downWeight + MXup*refP_upWeight;
     
     MZdown = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsMZ{1,1,refP_down,refIA_down}),controlListPacejkaMZ);
     MZup = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsMZ{1,1,refP_up,refIA_down}),controlListPacejkaMZ);
     MZ = MZdown*refP_downWeight + MZup*refP_upWeight;
else
    % no interpolation for pressure
end


% 3rd Interpolation - IA
if ~strcmp(refP_up,"off")
%      FYdown = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsFY{1,1,refP_down,refIA_down}),controlListPacejkaFY);
     FYdown = FY;
     FYup = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsFY{1,1,refP_down,refIA_up}),controlListPacejkaFY);
     FY = FYdown*refIA_downWeight + FYup*refIA_upWeight;
     
 %      MXdown = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsMX{1,1,refP_down,refIA_down}),controlListPacejkaMX);
     MXdown = MX;
     MXup = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsMX{1,1,refP_down,refIA_up}),controlListPacejkaMX);
     MX = MXdown*refIA_downWeight + MXup*refIA_upWeight;
     
 %      FYdown = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsMZ{1,1,refP_down,refIA_down}),controlListPacejkaMZ);
     MZdown = MZ;
     MZup = tyreLoadCalculator(alpha,FZ,FZ0,coeffvalues(sfitsMZ{1,1,refP_down,refIA_up}),controlListPacejkaMZ);
     MZ = MZdown*refIA_downWeight + MZup*refIA_upWeight;
else
    % no interpolation for IA
end

FY = FY*lambda_muy;
MX = MX*lambda_muy;
MZ = MZ*lambda_muy;
SA = alpha;


%% LONGITUDINAL
% 1. Evaluate tyre properties
compound_lon = t.compound_lon;

% 2. Get weights of interpolation
iTyre = find(strcmp({models.data.lonRefsFX(:).name},compound_lon));
lonRefs = models.data.lonRefsFX(iTyre); % get the reference (lon) data for the desired compound

% IA
if sum(IA == lonRefs.ref.IA) == 1 % if exactly matches one of the references, use exaclty that data
    refIA_down = find(IA == lonRefs.ref.IA);
    refIA_up = "off";
else
    for ii = 1:length(lonRefs.ref.IA)
        if ii == length(lonRefs.ref.IA) % if it is greater than the biggest IA tested, use the biggest IA
            refIA_down = ii;
            refIA_up = "off";
            break
        end
        if lonRefs.ref.IA(ii+1) > IA % if lies somewhere in between, interpolate solutions
            refIA_down = ii;
            refIA_up = ii+1;
            % linear interpolation: DOWN*x_1 + UP*(1-x_1) = IA; x_1 = (UP-IA)/(UP-DOWN)
            refIA_downWeight = (lonRefs.ref.IA(refIA_up)-IA)/(lonRefs.ref.IA(refIA_up)-lonRefs.ref.IA(refIA_down));
            refIA_upWeight = 1-refIA_downWeight;
            break
        end
        
    end
end

% P
if sum(P == lonRefs.ref.P) == 1 % if exactly matches one of the references, use exaclty that data
    refP_down = find(P == lonRefs.ref.P);
    refP_up = "off";
else
    for ii = 1:length(lonRefs.ref.P)
        if ii == 1
            if lonRefs.ref.P(ii) > P % if it is smaller than the smallest P tested, use the smallest P
                refP_down = ii;
                refP_up = "off";
                break
            end
        end
        if ii == length(lonRefs.ref.P) % if it is greater than the biggest P tested, use the biggest P
            refP_down = ii;
            refP_up = "off";
            break
        end
        if lonRefs.ref.P(ii+1) > P % if lies somewhere in between, interpolate solutions
            refP_down = ii;
            refP_up = ii+1;
            % linear interpolation: DOWN*x_1 + UP*(1-x_1) = P; x_1 = (UP-P)/(UP-DOWN)
            refP_downWeight = (lonRefs.ref.P(refP_up)-P)/(lonRefs.ref.P(refP_up)-lonRefs.ref.P(refP_down));
            refP_upWeight = 1-refP_downWeight;
            break
        end
        
    end
end

% % V
% if sum(V == lonRefs.ref.V) == 1 % if exactly matches one of the references, use exaclty that data
%     refV_down = find(V == lonRefs.ref.V);
%     refV_up = "off";
% else
%     for ii = 3:length(lonRefs.ref.V) % starting from 3 since 1,2 values for V are extremely small -no data available-
%         if ii == 3
%             if lonRefs.ref.V(ii) > V % if it is smaller than the smallest V tested, use the smallest V
%                 refV_down = ii;
%                 refV_up = "off";
%                 break
%             end
%         end
%         if ii == length(lonRefs.ref.V) % if it is greater than the biggest V tested, use the biggest V
%             refV_down = ii;
%             refV_up = "off";
%             break
%         end
%         if lonRefs.ref.V(ii+1) > V % if lies somewhere in between, interpolate solutions
%             refV_down = ii;
%             refV_up = ii+1;
%             % linear interpolation: DOWN*x_1 + UP*(1-x_1) = V; x_1 = (UP-V)/(UP-DOWN)
%             refV_downWeight = (lonRefs.ref.V(refV_up)-V)/(lonRefs.ref.V(refV_up)-lonRefs.ref.V(refV_down));
%             refV_upWeight = 1-refV_downWeight;
%             break
%         end
%         
%     end
% end

% check (only valid if values lying between two reference data points)
% latRefs.ref.IA(refIA_down)*refIA_downWeight+latRefs.ref.IA(refIA_up)*refIA_upWeight
% latRefs.ref.P(refP_down)*refP_downWeight+latRefs.ref.P(refP_up)*refP_upWeight
% latRefs.ref.V(refV_down)*refV_downWeight+latRefs.ref.V(refV_up)*refV_upWeight


% 1st Interpolation - V
% Note that there are only 3 values for the velocity, and only the central one has been tested with different values of IA, p
% First identify the lower and upper velocity sfits
% NOT IMPLEMENTED YET


% 2nd Interpolation - P
iV = 4;
sfits = models.data.lonModelsFX(iTyre,iV,:,:); % remove compound and velocity (the first two indexes must be == 1 from now on
controlListPacejka = models.data.lonRefsFX.controlListCoeffs;

% sigma = -0.3:0.01:0.3;
FZ0 = lonRefs.ref.FZ(5);

if ~strcmp(refP_up,"off")
     FXdown = tyreLoadCalculator(sigma,FZ,FZ0,coeffvalues(sfits{1,1,refP_down,refIA_down}),controlListPacejka);
     FXup = tyreLoadCalculator(sigma,FZ,FZ0,coeffvalues(sfits{1,1,refP_up,refIA_down}),controlListPacejka);
     FX = FXdown*refP_downWeight + FXup*refP_upWeight;
else
    % no interpolation for pressure
end


% 3rd Interpolation - IA
if ~strcmp(refP_up,"off")
%      FXdown = tyreLoadCalculator(sigma,FZ,FZ0,coeffvalues(sfits{1,1,refP_down,refIA_down}),controlListPacejka);
     FXdown = FX;
     FXup = tyreLoadCalculator(sigma,FZ,FZ0,coeffvalues(sfits{1,1,refP_down,refIA_up}),controlListPacejka);
     FX = FXdown*refIA_downWeight + FXup*refIA_upWeight;
else
    % no interpolation for IA
end

FX = FX*lambda_muy;
SL = sigma;

FX((SL < 0 & FX > 0) | (SL > 0 & FX < 0)) = 0; % Do not allow reversed sign in FX (by definition, the curve goes through (0,0), so reversed sign only possible due to model flaw at big abs(SL)
% Should be improved in terms of MAX SL for those conditions

%% TRACTION ELLIPSE

% a = (max(FY)-min(FY))/2;
% b = (max(FX)-min(FX))/2;
% x_0 = (max(FY)+min(FY))/2;
% y_0 = (max(FX)+min(FX))/2;
% 
% x = linspace(-a,a,100);
% y1 = b/a*sqrt(a^2-x.^2) + y_0;
% y2 = -b/a*sqrt(a^2-x.^2) + y_0;
% x = x + x_0;
% 
% figure
% plot(x,y1); hold on;
% plot(x,y2); hold on;
% plot(x_0,y_0,'k*');
% xlabel('F_y [N]')
% ylabel('F_x [N]')
% grid on
% axis equal

end