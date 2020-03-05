function [a, FXaero, FZaero] = aeroMap(a,V,rho)

% SCDf = 3.51;
% SCD = 1.43;

rhr = 32.5;
rhf = 37.5;

SCD = interp2(a.SCDmap(2:end,1),a.SCDmap(1,2:end),a.SCDmap(2:end,2:end).',rhr,rhf,'linear');
SCDf1 = interp2(a.SCDf1map(2:end,1),a.SCDf1map(1,2:end),a.SCDf1map(2:end,2:end).',rhr,rhf,'linear');
SCDf2 = interp2(a.SCDf2map(2:end,1),a.SCDf2map(1,2:end),a.SCDf2map(2:end,2:end).',rhr,rhf,'linear');
SCDf = SCDf1 + SCDf2;

FXaero = -1/2*rho*V.^2*SCD;
FZaero = 1/2*rho*V.^2*SCDf;

end
