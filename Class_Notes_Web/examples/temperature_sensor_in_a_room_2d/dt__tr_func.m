function dT__tr=dT__tr_func(t,T_min,T_max)

dT__tr=-(T_max-T_min)/2*1/(24*60*60)*cos(t/(24*60*60)*2*pi+pi/2);

end