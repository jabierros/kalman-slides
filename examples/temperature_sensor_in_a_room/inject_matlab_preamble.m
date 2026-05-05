figure('visible','off');
set(gcf, 'ToolBar', 'none');

warning('off','MATLAB:print:AxesToolbarInExport');
% 1. Set all text (Labels, Titles, Legends) to use LaTeX by default

set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
% 2. Set the Axes (Tick labels) to use LaTeX by default
set(groot, 'DefaultAxesTickLabelInterpreter', 'latex');

% 3. Set Font Units and Size to be proportional (Normalized)
set(groot, 'DefaultAxesFontUnits', 'normalized');
set(groot, 'DefaultAxesFontSize', 0.05);

% 4. Set default Line Width for all plots
set(groot, 'DefaultLineLineWidth', 2);
