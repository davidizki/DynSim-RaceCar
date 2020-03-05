function s = complianceCalculator(e, i, s)

% everything is output
s.krF = (i.msF.*e.g/2)/s.jounceMeasuredF + (i.muF.*e.g/2)/s.jounceMeasuredF; % [N/m] Front ride stiffness (using both jounce contributions)
s.krR = (i.msR.*e.g/2)/s.jounceMeasuredR + (i.muR.*e.g/2)/s.jounceMeasuredR; % [N/m] Rear "

s.kwF = (1./s.krF - 1/s.ktF).^-1; % [N/m] Front wheel centre stiffness
s.kwR = (1./s.krR - 1/s.ktR).^-1; % [N/m] Rear "

s.kcF = (1./s.kwF - g.mrF^2./s.ksF).^-1; % [N/m] Front compliance stiffness
s.kcR = (1./s.kwR - g.mrR^2./s.ksR).^-1; % [N/m] Rear "

end