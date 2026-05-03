inject_MATLAB_preamble;

global t sigma_T__meas T_min T_max
sigma_T__meas=4;
T_min=18; T_max=38;

global z__tr_func
z__tr_func=@(t) T__tr_func(t,T_min,T_max);


% mu_x_0 is the initial state estimation
mu_x_0=(50-(-5))/2 %ºC

% sigma_x_0 is the STD of the initial state estimation
sigma_x_0=50-mu_x_0; %ºC
sigma2_x_0=sigma_x_0^2;

sigma_w=20/(3600) %ºC/s
sigma2_w=sigma_w^2 %K^2

sigma_z__meas=[sigma_T__meas] %ºC
sigma2_z__meas=sigma_z__meas^2;

sigma_v=0;
sigma2_v=sigma_v^2 %ºC^2

fid=fopen('sol.dat','w');

mu_x=mu_x_0;
sigma2_x=sigma2_x_0; %K^2

t_series=(0:24*3600);

t=t_series(1);
z__meas=get_z();

fprintf(fid,'%d %d %d %d %d\n',t,mu_x,sigma2_x,z__meas, z__tr_func(t));

for k=1:length(t_series)-1

% Prediction: state @ k+1 using process model

mu_x_pred=mu_x;
sigma2_x=sigma2_x+sigma2_w;
I_x_pred=1/sigma2_x;

% Observation: state @ using sensor equation

t=t_series(k+1);
z__meas=get_z();
mu_x_obs=z__meas;
sigma2_x_obs=sigma2_z__meas + sigma2_v;
I_x_obs=1/sigma2_x_obs;

% Correction: state @ k+1 using ML fusion of estimations

I_total=I_x_pred+I_x_obs;
mu_x=(mu_x_pred*I_x_pred+mu_x_obs*I_x_obs)/I_total;
sigma2_x=1/I_total;

fprintf(fid,'%d %d %d %d %d\n',t,mu_x,sigma2_x,z__meas,z__tr_func(t));

end

fclose(fid)

load -ascii sol.dat ;

t_series=sol(:,1);
mu_x_series=sol(:,2);
sigma2_x_series=sol(:,3);
sigma_x_series=sqrt(sigma2_x_series);
sigma_z_series=ones(size(sigma_x_series))*sigma_z__meas;
z__meas_series=sol(:,4);
z__tr_series=sol(:,5);
t_series=sol(:,1);

fig = figure('visible','off');
plot(t_series,z__meas_series,t_series,mu_x_series,t_series,z__tr_series)
title_handle = title("$z^{meas}$ vs $\mu_x$ vs $z^{tr}$");
legend_('$z^{meas}$','$\mu_x$','$z^{t}$')
xlabel('$t$');
ylabel('$[^oC]$');

print(fig, 'plot1.png', '-dpng')
print(fig, 'plot1.pdf', '-dpdf')

fig = figure('visible','off');
plot(t_series,sigma_x_series,t_series,sigma_z_series)
title_handle = title("$\sigma_x$ vs $\sigma_{\mu_x}$");
legend_('$\sigma_x$','$\sigma_{z}$')
xlabel('$t$');
ylabel('$[^oC]$');

print(fig, 'plot2.png', '-dpng')
print(fig, 'plot2.pdf', '-dpdf')

fig = figure('visible','off');
plot(t_series,z__tr_series)
title_handle = title("$T^{tr}$");
legend_('$T^{tr}$')
xlabel('$t$');
ylabel('$[^oC]$');

print(fig, 'plot.png', '-dpng')
print(fig, 'plot.pdf', '-dpdf')