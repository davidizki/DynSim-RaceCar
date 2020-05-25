function [RR, MY] = RRcalculator(FZ,V,lambda_muy,t,alpha,sigma)

RR(V==0) = 0;

f = (0.005 + 100./t.pressure_kPa.*(0.01 + 0.12*(V/100).^2)) + 0.06/0.1.*abs(sigma-0.05);
RR(V~=0) = -lambda_muy.*f.*FZ;
RR = RR(:); % ensure it is a column

MY = -RR.*t.RL;

end