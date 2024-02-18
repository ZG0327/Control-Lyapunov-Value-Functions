close all
clear 
clc


%% Grid
grid_min = [-1; -1]; % Lower corner of computation domain
grid_max = [1; 1];    % Upper corner of computation domain
N = [151; 151];         % Number of grid points per dimension
% pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, []);

%% time vector
t0 = 0;
tMax = 40;
dt = 0.1;
tau = t0:dt:tMax;

%% problem parameters
uMode = 'min';
dMode = 'max';



%% Dynamics
uRange = [-1,1]; % Velocity of the Dubins car
dRange = [-0.5,0.5]; % Desired postion
dCar = Cart2D([0, 0], uRange, dRange);
gamma = 0;

%% Pack problem parameteres
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'veryHigh'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
schemeData.clf.gamma = gamma;

%% Compute the CLVF using algorithm 1. 
% First find the smallest value for the value function with gamma = 0
% Test with different cost functions:
% 1: l(x) = ||x||_2, 
% 2: l(x) = ||x||_infty
% 3: l(x) = x'Qx

% data0 = shapeRectangleByCorners(g, [0;0], [0;0]);
% data0 = shapeCylinder(g, [], [0; 0], 0); 

Q = [0.2,0;0,1];
data0 = QuadCost(g,Q);

[V_0,tau1] = ComputeHJ(data0,data0,tau,schemeData);
c = min(V_0,[],'all');

% save('V_gamma=0_fine.mat','V_1')
% save('g_fine.mat','g')
%% Compute CLVF with gamma \neq 0
% The cost function l(x) = data0 - c
% but we can warm start with IC = V_0 - c 

% data0_2 = data0-2;
% [V_2,tau2] = ComputeHJ(data0_2,tau,schemeData);
% % save('V_gamma=03_fine.mat','V_2')



%%
function [data,tau] = ComputeHJ(data0,cost,tau0,schemeData)
% data0: initial value function
% cost: cost function
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.valueFunction = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.deleteLastPlot = true; 
HJIextraArgs.convergeThreshold = 0.0001;
HJIextraArgs.stopConverge = 1;
HJIextraArgs.keepLast = 1;
HJIextraArgs.makeVideo = 0;
HJIextraArgs.targetFunction = cost;
% HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; 
% HJIextraArgs.visualize.plotData.projpt = {'min'}; 
HJIextraArgs.visualize.viewAngle = [30,45];

[data, tau, ~] = ...
  HJIPDE_ZGsolve(data0, tau0, schemeData, 'minCLF', HJIextraArgs);

end