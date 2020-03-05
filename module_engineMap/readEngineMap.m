
name = 'XXX';


filename = ['engineCurves_digitized_',name,'XXX.txt'];
engineCurves = csvread(filename);
engineCurves(:,3) = engineCurves(:,1).*engineCurves(:,2)*2*pi/60;

filename_out = ['engineCurves_',name,'XXX.txt'];
save(filename_out,'engineCurves');