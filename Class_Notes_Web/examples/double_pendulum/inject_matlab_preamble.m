%figure('visible','off');
set(groot, 'defaultFigureToolBar', 'none');

warning('off','MATLAB:print:AxesToolbarInExport');

% 1. Set all text (Labels, Titles, Legends) to use LaTeX by default
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');

% 2. Lock units to 'points' to prevent disproportionate scaling
set(groot, 'DefaultAxesFontUnits', 'points');
set(groot, 'DefaultTextFontUnits', 'points');
set(groot, 'DefaultLegendFontUnits', 'points');

% 3. Set a uniform Font Size for all elements
uniform_fontsize = 14; % Adjust this single number to scale the whole plot's text
set(groot, 'DefaultAxesFontSize', uniform_fontsize);
set(groot, 'DefaultTextFontSize', uniform_fontsize);
set(groot, 'DefaultLegendFontSize', uniform_fontsize);

% 4. Set default Line Width for all plots
set(groot, 'DefaultLineLineWidth', 1);