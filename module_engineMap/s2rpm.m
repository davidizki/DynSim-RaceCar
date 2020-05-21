function rpm = s2rpm(p,speed,gear,R)

rpm = speed./R.*(p.ratios(gear).').*(60/2/pi);

end