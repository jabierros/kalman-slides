inject_MATLAB_preamble;

global t sigma_T__meas T_min T_max
sigma_T__meas=4;
T_min=18; T_max=38;

global z__tr_func
z__tr_func=@(t) T__tr_func(t,T_min,T_max);
dz__tr_func=@(t) dT__tr_func(t,T_min,T_max);

% mu_x_0 is the initial state estimation
T_max_lim=50;
T_min_lim=-5;
T_0=(T_max_lim+T_min_lim)/2 
dT_0=0 %ºC/s
mu_x_0=[T_0;0];

% sigma_x_0 is the STD of the initial state estimation

% $ \dot{T}^{max}=\frac{20}{3600} ^\circ\text{C/s}$
% $\omega=\frac{2\pi}{24*3600} \text{ rad/s}$
% $\ddot{T}^{max}=\omega  \dot{T}^{max}$

sigma_T_0=(T_max_lim-T_min_lim)/2;
sigma_dT_0=20/3600 %ºC/s
Sigma_x_0=[sigma_T_0; sigma_dT_0]
Sigma2_x_0=diag(Sigma_x_0.^2)
omega=2*pi/(24*3600) % 2*pi/1 day
A_T=(T_max_lim-T_min_lim)/2
Delta_t=1 %s
sigma_w_T= 1/2 * 1/sqrt(2) * A_T*omega^2 * Delta_t^2
sigma_w_dT= 10*      1/sqrt(2) * A_T*omega^2 * Delta_t
Sigma_w=[sigma_w_T;sigma_w_dT]
Sigma2_w=diag(Sigma_w.^2)
Sigma_z=[sigma_T__meas]
Sigma2_z=diag(Sigma_z.^2);
Sigma_v=0;
Sigma2_v=Sigma_v^2
fid=fopen('sol.dat','w');
mu_x=mu_x_0;
Sigma2_x=Sigma2_x_0; %K^2
t_series=(0:Delta_t:24*3600);
t=t_series(1);
z=get_z();
f_x=[1,Delta_t;
     0,1];
h_x=[1,0];

fprintf(fid,'%d %d %d %d %d %d %d %d\n',t,mu_x,diag(Sigma2_x),z,z__tr_func(t),dz__tr_func(t));
for k=1:length(t_series)-1

% Prediction: state @ k+1 using process model
mu_x_pred=f_x*mu_x;
Sigma2_x_pred=f_x*Sigma2_x*f_x'+Sigma2_w;
i_x_pred=inv(Sigma2_x_pred)*mu_x_pred;
I_x_pred=inv(Sigma2_x_pred);

% Observation: state @ using sensor equation
t=t_series(k+1);
z=get_z();
h_x_mu_x_obs=z;
Sigma2_h_x_x_obs=Sigma2_z + Sigma2_v;
i_x_obs=h_x'*inv(Sigma2_h_x_x_obs)*h_x_mu_x_obs;
I_x_obs=h_x'*inv(Sigma2_h_x_x_obs)*h_x;

% Fusion: state @ k+1 using ML fusion of estimations
I_total=I_x_pred+I_x_obs;
mu_x=inv(I_total)*(i_x_pred+i_x_obs);
Sigma2_x=inv(I_total);

fprintf(fid,'%d %d %d %d %d %d %d %d\n',t,mu_x,diag(Sigma2_x),z,z__tr_func(t),dz__tr_func(t));
end

fclose(fid)

load -ascii sol.dat ;
t_series=sol(:,1);
mu_x_series=sol(:,2:3);
sigma2_x_series=sol(:,4:5);
sigma_x_series=sqrt(sigma2_x_series);
sigma_z_series=ones(length(sigma_x_series),1)*Sigma_z;
z_series=sol(:,6);
z__tr_series=sol(:,7);
dz__tr_series=sol(:,8);
t_series=sol(:,1);
fig = figure('visible','on');
plot(t_series,z_series,t_series,mu_x_series(:,1),t_series,z__tr_series)
title_handle = title("$z^{meas}$ vs $\mu_x(1)$ vs $z^{tr}$");
legend_('$z^{meas}$','$\mu_x(1)$','$z^{t}$')
xlabel('$t$');
ylabel('$[^oC]$');
print(fig, 'plot1.png', '-dpng')
print(fig, 'plot1.pdf', '-dpdf')
fig = figure('visible','on');
plot(t_series,mu_x_series(:,2),t_series,dz__tr_series)
title_handle = title(" $\mu_x(2)$ vs $\dot{z}^{tr}$");
legend_('$\mu_x(2)$','$\dot{z}^{tr}$')
xlabel('$t$');
ylabel('$[^oC]$');
print(fig, 'plot1.2.png', '-dpng')
print(fig, 'plot1.2.pdf', '-dpdf')
fig = figure('visible','on');
plot(t_series,sigma_x_series(:,1),t_series,sigma_x_series(:,2),t_series,sigma_z_series)
title_handle = title("$\sigma_x$ vs $\sigma_{\mu_x}$");
legend_('$\sigma_x(1)$','$\sigma_x(2)$','$\sigma_{z}$')
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