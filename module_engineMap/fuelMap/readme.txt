1st Open the .pclr file. Go to "Tuning". Right click on the map -> Import/Export -> Export to file (.ite)
2nd Copy the values in the file to MATLAB, using the following names for each variable:
fuelMap.rpm
fuelMap.delta_t_real
fuelMap.Z
Z is a vector ordered as follows: it "runs along rpm" (for a given delta_t_real) 

3rd Execute: save('fuelMap_XXXXXXXX.mat','fuelMap')
4th Move it to the fuelMap folder