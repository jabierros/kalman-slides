function T__tr=T__tr_func(t,T_min,T_max)

T__tr=(T_max+T_min)/2-(T_max-T_min)/2*sin(t/(24*60*60)*2*pi+pi/2);

end