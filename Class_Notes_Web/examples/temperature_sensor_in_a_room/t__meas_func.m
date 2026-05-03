function T__meas=T__meas_func(t,sigma_T__meas,T_min,T_max)

T__tr=T__tr_func(t,T_min,T_max);
T__meas=T__tr+randn*sigma_T__meas;

end
