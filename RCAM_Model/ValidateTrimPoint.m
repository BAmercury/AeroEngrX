% Based on Chris Lum's tutorial: https://www.youtube.com/watch?v=YzZI1V2mJw8
clear; clc; clear all

% Load the trim point save file
temp = load('trim_values_sl');
XStar = temp.XStar;
UStar = temp.UStar;

TF = 250;

% Define min/max control limits
% Control Limits and Saturations definitions

% Aileron
u1min = -25*(pi/180);
u1max = 25*(pi/180);

% HStab
u2min = -25*(pi/180);
u2max = 10*(pi/180);

% Rudder
u3min = -30*(pi/180);
u3max = 30*(pi/180);

% Throttle 1
%u4min = 0.5*(pi/180);
u4min = 0; % Engine Shutoff Test
u4max = 10*(pi/180);

% Throttle 2
u5min = 0.5*(pi/180);
u5max = 10*(pi/180);


% Run Model
tic
out = sim('RCAM_Sim_TrimTest')
toc

% Extract the data
t = out.simX.Time;
X = out.simX.Data;

figure;
for k = 1:9
    subplot(5,2,k)
    plot(t, X(:,k), 'LineWidth', 2)
    ylabel(['x_', num2str(k)])
    grid on
end

disp('Done')