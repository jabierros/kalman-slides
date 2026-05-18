clear all; clc;


global t t_0 t_end Delta_t

global mu_x Sigma2_x

global u_meas z_meas

global sigma_u_actual sigma_z_actual x_actual x_actual_0 u_actual_func rng_status seed

global x_string u_string z_string

q_string=["\theta_1";"\theta_2"];
dq_string=["\dot{\theta}_1";"\dot{\theta}_2"];
x_string=[q_string;
          dq_string];
u_string=[];
z_string=["\omega_y";"a_x";"a_z"];

global param

seed=1789; rng(seed);

rng_status=rng;

% Define handle to function determining the actual input

u_actual_func = @(t) my_u_actual_func(t);

t_0=0; %s

Delta_t=0.001; %s

t_end=50; %s

m1=1 %kg

m2=1 %kg

l1=1 %m

l2=1 %m

g=10 %m/s^2

param=[m1 m2 l1 l2 g Delta_t]' % as is from main_symbolic_EKF.m

theta1_0=pi/6 %rad

dtheta1_0=0 %rad/s

theta2_0=pi/6 %rad

dtheta2_0=0 %rad/s

q_0=[theta1_0,theta2_0]'

dq_0=[dtheta1_0,dtheta2_0]'

x_actual_0=[q_0;dq_0]

sigma_theta1_0=pi/2 % rad no idea at all

sigma_dtheta1_0=0.1 % rad/s no idea at all

sigma_theta2_0=pi/2 % rad no idea at all

sigma_dtheta2_0=0.1 % rad/s no idea at all

sigma_x_0= [sigma_theta1_0,sigma_theta2_0,sigma_dtheta1_0,sigma_dtheta2_0]';

mu_x_0 = x_actual_0+sigma_x_0;

Sigma2_x_0 = diag(sigma_x_0.^2); % assumed diagonal @ t=t_0

max_error_discr=[2.58418e-06, 3.025e-06, 1.08183e-05, 2.12142e-05]'

sigma_discr=max_error_discr; %discr. err. cov. assumed diagonal

n_x=size(x_actual_0,1);
sigma_w_x=zeros([n_x,1]);


n_u=size(u_actual_func(t_0),1);
sigma_u_actual=zeros(n_u,1);
sigma_u=zeros(n_u,1); %input meas. cov. assumed diagonal

sigma_gyroy_actual = 0.1;%rad/s
sigma_accx_actual  = 0.1;%m/s^2
sigma_accz_actual  = 0.1;%m/s^2

sigma_gyroy_spec=sigma_gyroy_actual;
sigma_accx_spec=sigma_accx_actual;
sigma_accz_spec=sigma_accz_actual;

sigma_z_actual=[sigma_gyroy_actual,sigma_accx_actual,sigma_accz_actual]';

sigma_z=[sigma_gyroy_spec,sigma_accx_spec,sigma_accz_spec]';

n_z=size(sigma_z_actual,1);
sigma_v_x=zeros(n_z,1);

t=t_0;

t=t_0;

mu_x=mu_x_0;

Sigma2_x=Sigma2_x_0;

sigma_x=sigma_x_0;

x_actual=x_actual_0;

datalogging_string={'t';'x_actual';'mu_x';'u_actual';'u_meas';'z_actual';'z_meas';'sigma_x'};
fid=fopen('sol.dat','w');
rng(rng_status);


u_meas=get_u(); z_meas=get_z();

datalogging(fid, datalogging_string);

for k=1:t_end/Delta_t

IEKF(sigma_discr, sigma_z, sigma_u,sigma_w_x,sigma_v_x);

datalogging(fid, datalogging_string)

end

u_meas=get_u(); z_meas=get_z();

f_x_=f_x(mu_x,u_meas,t,param);

h_x_=h_x(mu_x,u_meas,t,param);

OB=obsv(f_x_,h_x_)

rank(OB)

size(OB)

load_datalogging('sol.dat', datalogging_string);

k=length(t_series)-5;

OB=[h_x(x_actual_series(k,:)',u_meas_series(k,:)',t_series(k,:),param)

h_x(x_actual_series(k+1,:)',u_meas_series(k+1,:)',t_series(k+1,:),param)*f_x(x_actual_series(k+1,:)',u_meas_series(k+1,:)',t_series(k+1,:),param)

h_x(x_actual_series(k+2,:)',u_meas_series(k+2,:)',t_series(k+2,:),param)*f_x(x_actual_series(k+2,:)',u_meas_series(k+2,:)',t_series(k+2,:),param)*f_x(x_actual_series(k+1,:)',u_meas_series(k+1,:)',t_series(k+1,:),param)

h_x(x_actual_series(k+3,:)',u_meas_series(k+3,:)',t_series(k+3,:),param)*f_x(x_actual_series(k+3,:)',u_meas_series(k+3,:)',t_series(k+3,:),param)*f_x(x_actual_series(k+2,:)',u_meas_series(k+2,:)',t_series(k+2,:),param)*f_x(x_actual_series(k+1,:)',u_meas_series(k+1,:)',t_series(k+1,:),param)

h_x(x_actual_series(k+4,:)',u_meas_series(k+4,:)',t_series(k+4,:),param)*f_x(x_actual_series(k+4,:)',u_meas_series(k+4,:)',t_series(k+4,:),param)*f_x(x_actual_series(k+3,:)',u_meas_series(k+3,:)',t_series(k+3,:),param)*f_x(x_actual_series(k+2,:)',u_meas_series(k+2,:)',t_series(k+2,:),param)*f_x(x_actual_series(k+1,:)',u_meas_series(k+1,:)',t_series(k+1,:),param)]

rank(OB)

clear *_series



load_datalogging('sol.dat', datalogging_string)

mu_x_error_series=mu_x_series-x_actual_series;

num_samples_statistic=100;

lim_mu_x_error=mu_x_error_series(end-num_samples_statistic:end,:);

lim_mu_x_error_mean=mean(lim_mu_x_error)

lim_mu_x_error_std=std(lim_mu_x_error)

sqrt_lim_mu_x_error_squared_mean=mean((lim_mu_x_error).^2).^0.5



load_datalogging('sol.dat', datalogging_string);

fig_dir=['Information_EKF_',num2str(Delta_t)];

Plotting(fig_dir, datalogging_string);


function u_actual=my_u_actual_func(t)
u_actual=zeros(0,1);
end