function speed = rpm2s(p,rpm,gear,R)


speed = rpm.*(2*pi/60)./(p.ratios(gear).').*R;


end