close all
clear 
clc



%% Grid
grid_min = [-3,-3]; % Lower corner of computation domain
grid_max = [3,3];   % Upper corner of computation domain
N = 121;         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N);

%% target set
R = 0;
data0 = shapeCylinder(g, [],[0,0],R);
%% time vecto

t0 = 0;
tMax = 30;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% control trying to min or max value function?
uMode = 'min';
dMode = 'max';



%% Pack problem parameters

%obj = nvertedPendulum(x0, params)

x0 = [0,0];
params.u_min = -2;
params.u_max = 2;
params.l = 1;
params.m = 1;   
params.g = 1; 
params.b = 1; 
dyn = InvertedPendulum(x0, params);

gamma1 = 0;
gamma2 = 0.5;
gamma3 = 0.001;

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dyn;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;



%% Compute value function

[data1,tau1] = ComputeHJ(data0,tau,schemeData,1,gamma2);


% %%
save('value.mat','data1');
save('grid','g');
save('params','params')


%% HJB solver
function [data,tau] = ComputeHJ(data0,tau0,schemeData,n,gamma)

HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.valueFunction = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.viewAngle = [30,45,30]
HJIextraArgs.visualize.figNum = n; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.targetFunction = data0;
HJIextraArgs.convergeThreshold = 0.003;
HJIextraArgs.stopConverge = 1;
HJIextraArgs.keepLast = 1;
HJIextraArgs.makeVideo = 0;
HJIextraArgs.ignoreBoundary=1;
schemeData.clf.gamma = gamma;

% [data, tau, ~] = ...
%   HJIPDE_ZGsolve(data0, tau0, schemeData,'none', HJIextraArgs);
[data, tau, ~] = ...
    HJIPDE_ZGsolve(data0, tau0, schemeData,'minCLF', HJIextraArgs);
end
