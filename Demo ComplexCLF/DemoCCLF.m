close all
clear 
clc

%% Grid
grid_min = [-3,-3]; % Lower corner of computation domain
grid_max = [3,3];   % Upper corner of computation domain
N = 101;         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N);

%% target set
R = 0;
data0 = shapeCylinder(g, [],[0,0],R);
% data0 = shapeRectangleByCorners(g, [-3;-50], [3;50]);
% data00 = data0*0.5;
% data00 = data0 + 1.19;
% data01 = min(data0,-1.5);
%% time vecto

t0 = 0;
tMax = 20;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% control trying to min or max value function?
uMode = 'min';
dMode = 'max';

params.uMin = -5;
params.uMax = 5;


%% Pack problem parameters


dyn = CCLF([0,0], params);

gamma1 = 0;
gamma2 = 0.2;
gamma3 = 2;

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dyn;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;



%% Compute value function
% 
[data1,tau1] = ComputeHJ(data0,tau,schemeData,1,gamma2);
% [data2,tau2] = ComputeHJ(data0,tau,schemeData,2,gamma2);
% [data3,tau3] = ComputeHJ(data0,tau,schemeData,3,gamma3);

%%
save('data1.mat','data1');
save('g.mat','g');
save('params','params')


%%
function [data,tau] = ComputeHJ(data0,tau0,schemeData,n,gamma)

HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.valueFunction = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.viewAngle = [30,45,30];
HJIextraArgs.visualize.figNum = n; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
% HJIextraArgs.visualize.sliceLevel = 0;

HJIextraArgs.targetFunction = data0;
HJIextraArgs.convergeThreshold = 1e-3;
HJIextraArgs.stopConverge = 1;
HJIextraArgs.keepLast = 1;
HJIextraArgs.makeVideo = 0;
schemeData.clf.gamma = gamma;
HJIextraArgs.divergeThreshold = 20;
HJIextraArgs.stopDiverge = 1;


% [data, tau, ~] = ...
%   HJIPDE_ZGsolve(data0, tau0, schemeData,'none', HJIextraArgs);
[data, tau, ~] = ...
    HJIPDE_ZGsolve(data0, tau0, schemeData,'minCLF', HJIextraArgs);
end
