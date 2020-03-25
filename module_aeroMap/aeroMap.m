function [a, FXaero, FZaeroF, FZaeroR, MYaero] = aeroMap(g,a,V,rho)

% SCDf = 3.51;
% SCD = 1.43;

rhr = 32.5;
rhf = 37.5;

SCD = interp2(a.SCDmap(2:end,1),a.SCDmap(1,2:end),a.SCDmap(2:end,2:end).',rhr,rhf,'linear');
SCDf1 = interp2(a.SCDf1map(2:end,1),a.SCDf1map(1,2:end),a.SCDf1map(2:end,2:end).',rhr,rhf,'linear'); % front
SCDf2 = interp2(a.SCDf2map(2:end,1),a.SCDf2map(1,2:end),a.SCDf2map(2:end,2:end).',rhr,rhf,'linear'); % rear

FXaero = -1/2*rho*V.^2*SCD; % assumption: applied at the centre of gravity. Only downforce contributes to the aerodynamic moments
FZaeroF = 1/2*rho*V.^2*SCDf1;
FZaeroR = 1/2*rho*V.^2*SCDf2;

MYaero = -FZaeroF*g.wb*(1-g.wDistr) + FZaeroR*g.wb*g.wDistr;

a.SCD = SCD;
a.SCDf = SCDf1 + SCDf2;
a.SCDf1 = SCDf1;
a.SCDf2 = SCDf2;
a.Xcp = -MYaero./(FZaeroF + FZaeroR);

end
