clear all; clc;
cd(fileparts(which(mfilename)));  % In a .m file uncomment this
%cd(fileparts(matlab.desktop.editor.getActiveFilename)); % In a .mlx file uncomment this
inject_MATLAB_preamble
fig_visibility = 'on';

%% Numeric code for Double Pendulum (IEKF / Smoother)


% =========================================================================
% BLOCK 0: PARAMETERS & INITIALIZATION
% =========================================================================
% 1. Time Series Definition
t_0 = 0;
Delta_t = 0.001;
t_end = 25;
t_series = (t_0:Delta_t:t_end)';
N = length(t_series) - 1;

% 2. System Physical Parameters
param = [1; 1; 1; 1; 10.0]; % [m1; m2; l1; l2; g]
param_true = param; 

% 3. Ground Truth Initial Conditions
theta1_true_0  = pi/6; % rad
dtheta1_true_0 = 0;    % rad/s
theta2_true_0  = pi/6; % rad
dtheta2_true_0 = 0;    % rad/s

x_true_0 = [theta1_true_0; theta2_true_0; dtheta1_true_0; dtheta2_true_0];

% 4. Initial Belief
sigma_x_0  = [pi/2; pi/2; 0.1; 0.1];
mu_x_0     = x_true_0 + sigma_x_0; 
Sigma2_x_0 = diag(sigma_x_0.^2);

% 5. Noise Standard Deviations
n_u = 0;
sigma_u = zeros(n_u, 1);
sigma_w = [2.58e-06; 3.02e-06; 1.08e-05; 2.12e-05]; % Discretization error
sigma_z     = [0.1; 0.1; 0.1]; % Hardware sensor noise (gyro_y, acc_x, acc_z)
sigma_v = [0; 0; 0];

% Pre-compute constant covariance matrices (Mimics Library Initialization)
Sigma2_w = diag(sigma_w.^2);
Sigma2_v = diag(sigma_v.^2);
Sigma2_u = diag(sigma_u.^2);
Sigma2_z = diag(sigma_z.^2);

% 6. Input function (Unforced)
u_true_func_ = @(t) zeros(0, 1);

% =========================================================================
% BLOCK 1: THE PHYSICAL UNIVERSE (High-Fidelity Simulation)
% =========================================================================
n_x = length(x_true_0);
n_z = length(sigma_z);

x_true_series = zeros(N+1, n_x);
u_meas_series = zeros(N+1, n_u); 
z_meas_series = zeros(N+1, n_z); 

% Initialize Reality
x_true_series(1,:) = x_true_0';
u_true_series(1,:) = u_true_func_(t_series(1))';
u_meas_series(1,:) = u_true_series(1,:) + (randn(1, n_u) .* sigma_u');
z_true_0 = h_(x_true_series(1,:)', u_true_series(1,:)', t_series(1), param);
z_true_series(1,:) = z_true_0';
z_meas_series(1,:) = z_true_0' + (randn(1, n_z) .* sigma_z');

ode45_options = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);

disp('Simulating Physical Reality...');

for k = 1:N
    t_span = [t_series(k), t_series(k+1)];
    u_static = u_true_func_(t_series(k));
    ode_func = @(t, x) dstate_true_(x, u_static, t_series(k), param_true);
    [~, x_out] = ode45(ode_func, t_span, x_true_series(k, :)', ode45_options);
    
    x_true_series(k+1, :) = x_out(end, :);
    u_meas_series(k+1, :) = u_true_func_(t_series(k+1))' + (randn(1, n_u) .* sigma_u');
    
    z_true_kp1 = h_true_(x_true_series(k+1,:)', u_meas_series(k+1,:)', t_series(k+1), param_true);
    z_meas_series(k+1, :) = z_true_kp1' + (randn(1, n_z) .* sigma_z');
end

% =========================================================================
% BLOCK 2: THE ESTIMATOR (Explicit IEKF Loop)
% =========================================================================
mu_x_series = zeros(N+1, n_x);
Sigma2_x_series = zeros(N+1, n_x, n_x);

mu_x_series(1,:) = mu_x_0';
Sigma2_x_series(1,:,:) = Sigma2_x_0;

disp('Running Explicit Filter Loop...');
for k = 1:N
    t_k        = t_series(k);
    t_kp1      = t_series(k+1);
    mu_x_k     = mu_x_series(k, :)';
    Sigma2_x_k = squeeze(Sigma2_x_series(k, :, :));
    
    u_meas_k   = u_meas_series(k, :)';     
    u_meas_kp1 = u_meas_series(k+1, :)';   
    z_meas_kp1 = z_meas_series(k+1, :)';   

    % ---------------------------------------------------------------------
    % 2.1 PREDICTION PHASE
    % ---------------------------------------------------------------------
    mu_x_pred = f_(mu_x_k, u_meas_k, t_k, param, Delta_t);

    f_x = f_x_(mu_x_k, u_meas_k, t_k, param, Delta_t);
    f_u = f_u_(mu_x_k, u_meas_k, t_k, param, Delta_t);

    Sigma2_f_u_u = f_u * Sigma2_u * f_u'; 
    Sigma2_x_pred_noise = Sigma2_w + Sigma2_f_u_u; 

    Sigma2_x_pred = f_x * Sigma2_x_k * f_x' + Sigma2_x_pred_noise;
    I_x_pred      = inv(Sigma2_x_pred);
    i_x_pred      = I_x_pred * mu_x_pred;

    % ---------------------------------------------------------------------
    % 2.2 UPDATE PHASE
    % ---------------------------------------------------------------------
    h_pred = h_(mu_x_pred, u_meas_kp1, t_kp1, param);
    h_x    = h_x_(mu_x_pred, u_meas_kp1, t_kp1, param);
    h_u    = h_u_(mu_x_pred, u_meas_kp1, t_kp1, param);

    Sigma2_h_u_u = h_u * Sigma2_u * h_u'; 
    Sigma2_obs   = Sigma2_z + Sigma2_v + Sigma2_h_u_u;

    I_x_obs = h_x' * inv(Sigma2_obs) * h_x;
    i_x_obs = h_x' * inv(Sigma2_obs) * (z_meas_kp1 - (h_pred - h_x * mu_x_pred));

    % ---------------------------------------------------------------------
    % 2.3 FUSION
    % ---------------------------------------------------------------------
    I_x_updated = I_x_pred + I_x_obs;
    i_x_updated = i_x_pred + i_x_obs;

    Sigma2_x_updated = inv(I_x_updated);
    mu_x_updated     = Sigma2_x_updated * i_x_updated;

    mu_x_series(k+1, :) = mu_x_updated';
    Sigma2_x_series(k+1, :, :) = Sigma2_x_updated;
end

% =========================================================================
% BLOCK 3: PLOTTING & STATISTICAL ANALYSIS
% =========================================================================
disp('Generating plots...');
set(groot, 'DefaultFigureWindowStyle', 'docked'); 

% 1. Data Preparation & Normalization
sigma_x_series = zeros(N+1, n_x);
for k = 1:N+1
    sigma_x_series(k, :) = sqrt(diag(squeeze(Sigma2_x_series(k, :, :))))';
end

angle_indices = [1, 2];
x_plot = x_true_series;
mu_plot = mu_x_series;
for idx = angle_indices
    x_plot(:, idx)  = mod(x_plot(:, idx) + pi, 2*pi) - pi;
    mu_plot(:, idx) = mod(mu_plot(:, idx) + pi, 2*pi) - pi;
end

mu_x_error_series = mu_plot - x_plot;
mu_x_error_series(:, angle_indices) = mod(mu_x_error_series(:, angle_indices) + pi, 2*pi) - pi;

num_samples_statistic = 100;
lim_mu_x_error = mu_x_error_series(end-num_samples_statistic:end, :);
sqrt_lim_mu_x_error_squared_mean = sqrt(mean(lim_mu_x_error.^2));

x_string = {'\theta_1', '\theta_2', '\dot{\theta}_1', '\dot{\theta}_2'};

% -------------------------------------------------------------------------
% Figure: Estimates vs Reality
% -------------------------------------------------------------------------
figure('Name', 'Tracking Performance', 'Visible', fig_visibility); hold on; grid on;
plot(t_series, mu_plot, '-');
set(gca, 'ColorOrderIndex', 1); 
plot(t_series, x_plot, '--');
title('$\mathbf{x}^{tr}$ vs $\hat{\mu}_{\mathbf{x}}$');
xlabel('$t$ [s]');
legend([strcat('$\hat{\mu}_{', x_string, '}$'), strcat('$', x_string, '^{tr}$')],'NumColumns', 2);

% -------------------------------------------------------------------------
% Figure: Estimation Error (RMS comparison)
% -------------------------------------------------------------------------
figure('Name', 'Estimation Error', 'Visible', fig_visibility); hold on; grid on;
plot(t_series, mu_x_error_series, '-');
set(gca, 'ColorOrderIndex', 1);
plot(t_series, repmat(sqrt_lim_mu_x_error_squared_mean, N+1, 1), '--');
title('$\hat{\mu}_{\mathbf{x}} - \mathbf{x}^{tr}$');
xlabel('$t$ [s]');
legend([strcat('$\hat{\mu}_{', x_string, '} - ', x_string, '^{tr}$'), strcat('RMS $\epsilon_{', x_string, '}$')], 'NumColumns', 2);

% -------------------------------------------------------------------------
% Figure: Covariance Analysis (Sigma vs RMS)
% -------------------------------------------------------------------------
figure('Name', 'Covariance Analysis', 'Visible', fig_visibility); hold on; grid on;
plot(t_series, sigma_x_series, '-');
set(gca, 'ColorOrderIndex', 1);
plot(t_series, repmat(sqrt_lim_mu_x_error_squared_mean, N+1, 1), '--');
set(gca, 'YScale', 'log');
title('$\mathrm{diag}(\Sigma^2_{\mathbf{x}})^{1/2}$ vs Limiting RMS Error');
xlabel('$t$ [s]');
legend([strcat('$\sigma_{', x_string, '}$'), strcat('RMS $\epsilon_{', x_string, '}$')], 'NumColumns', 2);

disp('Plotting complete.');