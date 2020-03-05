1st column: crankshaft angular velocity [rpm]
2nd column: crankshaft torque [Nm]
3rd column: crankshaft power [W]

Dummy values at lower rpm are added to avoid interpolation errors (esp. when starting from a low speed)

To load a new curve:
1. Open the .txt and ADD VALUES IN THE UPPER LOWER RANGE (MADE UP) TO AVOID INTERPOLATION ERRORS
2. Open the .txt in MATLAB with csvread. Add the power column (in W! Change the omega units)
3. Save the variable. Change the name of the desired curve in the main code